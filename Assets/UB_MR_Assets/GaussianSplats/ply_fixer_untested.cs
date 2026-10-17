#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Rewrites a Gaussian Splatting PLY into the standard 3DGS layout expected by
/// UnityGaussianSplatting (62 floats / 248 bytes per vertex):
///   x y z nx ny nz f_dc_0..2 f_rest_0..44 opacity scale_0..2 rot_0..3
/// Missing properties (e.g. nx, ny, nz) are written as 0. Property order is fixed.
///
/// Put this file in any folder named "Editor" (e.g. Assets/Editor/), then use
/// Tools > Gaussian Splatting > Fix PLY Layout...
/// </summary>
public static class GaussianPlyFixer
{
    const int kChunkVertices = 65536;

    static readonly string[] kStandardLayout = BuildLayout();

    static string[] BuildLayout()
    {
        var list = new List<string> { "x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2" };
        for (int i = 0; i < 45; i++) list.Add("f_rest_" + i);
        list.AddRange(new[] { "opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3" });
        return list.ToArray();
    }

    [MenuItem("Tools/Gaussian Splatting/Fix PLY Layout...")]
    static void Run()
    {
        string input = EditorUtility.OpenFilePanel("Select PLY to fix", "", "ply");
        if (string.IsNullOrEmpty(input)) return;

        string defaultName = Path.GetFileNameWithoutExtension(input) + "_fixed.ply";
        string output = EditorUtility.SaveFilePanel("Save fixed PLY", Path.GetDirectoryName(input), defaultName, "ply");
        if (string.IsNullOrEmpty(output)) return;

        if (Path.GetFullPath(input) == Path.GetFullPath(output))
        {
            EditorUtility.DisplayDialog("Fix PLY", "Output must be a different file than the input.", "OK");
            return;
        }

        try
        {
            Convert(input, output);
            EditorUtility.DisplayDialog("Fix PLY", "Done:\n" + output, "OK");
        }
        catch (Exception e)
        {
            Debug.LogError("Fix PLY failed: " + e);
            EditorUtility.DisplayDialog("Fix PLY failed", e.Message, "OK");
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }
    }

    static void Convert(string inputPath, string outputPath)
    {
        using (var inStream = new FileStream(inputPath, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20))
        using (var outStream = new FileStream(outputPath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
        {
            // ---- Parse header ----
            string firstLine = ReadLine(inStream);
            if (firstLine != "ply")
                throw new Exception("Not a PLY file.");

            bool littleEndian = false;
            long vertexCount = 0;
            string currentElement = null;
            var props = new List<string>();   // vertex properties, in file order

            while (true)
            {
                string line = ReadLine(inStream);
                if (line == null) throw new Exception("Unexpected end of file in header.");
                if (line == "end_header") break;

                string[] t = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (t.Length == 0) continue;

                switch (t[0])
                {
                    case "format":
                        littleEndian = t.Length > 1 && t[1] == "binary_little_endian";
                        break;
                    case "element":
                        currentElement = t[1];
                        if (currentElement == "vertex") vertexCount = long.Parse(t[2]);
                        break;
                    case "property":
                        if (currentElement != "vertex") break;
                        if (t[1] != "float" && t[1] != "float32")
                            throw new Exception("Unsupported property type '" + t[1] + "' for '" + t[2] +
                                                "'. Only float properties are supported.");
                        props.Add(t[2]);
                        break;
                }
            }

            if (!littleEndian || !BitConverter.IsLittleEndian)
                throw new Exception("Only binary_little_endian PLY files are supported.");
            if (vertexCount <= 0 || props.Count == 0)
                throw new Exception("No vertex data found.");

            int srcCount = props.Count;

            // Map each standard property to its index in the source (or -1 if missing)
            int[] map = new int[kStandardLayout.Length];
            var missing = new List<string>();
            for (int i = 0; i < map.Length; i++)
            {
                map[i] = props.IndexOf(kStandardLayout[i]);
                if (map[i] < 0) missing.Add(kStandardLayout[i]);
            }

            // Only normals are safe to fill with zero; anything else missing is a real problem.
            foreach (string m in missing)
            {
                if (m != "nx" && m != "ny" && m != "nz")
                    throw new Exception("Source is missing required property '" + m + "'.");
            }

            int dstCount = kStandardLayout.Length; // 62

            long expectedBytes = vertexCount * srcCount * 4L;
            if (inStream.Length - inStream.Position < expectedBytes)
                throw new Exception("File is truncated: vertex data is smaller than the header says.");

            // ---- Write header ----
            var sb = new StringBuilder();
            sb.Append("ply\n");
            sb.Append("format binary_little_endian 1.0\n");
            sb.Append("element vertex ").Append(vertexCount).Append('\n');
            foreach (string name in kStandardLayout)
                sb.Append("property float ").Append(name).Append('\n');
            sb.Append("end_header\n");
            byte[] headerBytes = Encoding.ASCII.GetBytes(sb.ToString());
            outStream.Write(headerBytes, 0, headerBytes.Length);

            // ---- Convert in chunks ----
            byte[] srcBytes = new byte[kChunkVertices * srcCount * 4];
            float[] srcFloats = new float[kChunkVertices * srcCount];
            float[] dstFloats = new float[kChunkVertices * dstCount];
            byte[] dstBytes = new byte[dstFloats.Length * 4];

            long done = 0;
            while (done < vertexCount)
            {
                int n = (int)Math.Min(kChunkVertices, vertexCount - done);
                int byteCount = n * srcCount * 4;
                ReadExactly(inStream, srcBytes, byteCount);
                Buffer.BlockCopy(srcBytes, 0, srcFloats, 0, byteCount);

                for (int v = 0; v < n; v++)
                {
                    int sBase = v * srcCount;
                    int dBase = v * dstCount;
                    for (int p = 0; p < dstCount; p++)
                    {
                        int s = map[p];
                        dstFloats[dBase + p] = s >= 0 ? srcFloats[sBase + s] : 0f;
                    }
                }

                int outBytes = n * dstCount * 4;
                Buffer.BlockCopy(dstFloats, 0, dstBytes, 0, outBytes);
                outStream.Write(dstBytes, 0, outBytes);

                done += n;
                if (EditorUtility.DisplayCancelableProgressBar("Fixing PLY",
                        done + " / " + vertexCount + " splats", (float)done / vertexCount))
                    throw new OperationCanceledException("Cancelled.");
            }

            Debug.Log("Fix PLY: wrote " + vertexCount + " splats (" + dstCount * 4 +
                      " bytes each) to " + outputPath +
                      (missing.Count > 0 ? ". Filled with 0: " + string.Join(", ", missing) : ""));
        }
    }

    static string ReadLine(Stream s)
    {
        var sb = new StringBuilder();
        int b;
        while ((b = s.ReadByte()) >= 0)
        {
            if (b == '\n') return sb.ToString().TrimEnd('\r');
            sb.Append((char)b);
            if (sb.Length > 4096) throw new Exception("Header line too long; not a valid PLY.");
        }
        return sb.Length > 0 ? sb.ToString() : null;
    }

    static void ReadExactly(Stream s, byte[] buffer, int count)
    {
        int read = 0;
        while (read < count)
        {
            int r = s.Read(buffer, read, count - read);
            if (r <= 0) throw new EndOfStreamException("Unexpected end of file.");
            read += r;
        }
    }
}
#endif