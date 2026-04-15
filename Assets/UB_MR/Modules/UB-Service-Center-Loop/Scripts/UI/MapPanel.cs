using UnityEngine;
using TMPro;
using CAVAS.UI;

public class MapPanel : Panel
{
    [Header("Offsets")]
    [SerializeField] TMP_InputField pos_x;
    [SerializeField] TMP_InputField pos_y;
    [SerializeField] TMP_InputField pos_z;

    [Space]
    [SerializeField] TMP_InputField rot_x;
    [SerializeField] TMP_InputField rot_y;
    [SerializeField] TMP_InputField rot_z;

    // -------- GETTERS --------
    public Vector3 GetMapPosition()
    {
        return new Vector3(
            ParseFloat(pos_x.text),
            ParseFloat(pos_y.text),
            ParseFloat(pos_z.text)
        );
    }

    public Quaternion GetMapRotation()
    {
        return Quaternion.Euler(
            new Vector3(
                ParseFloat(rot_x.text),
                ParseFloat(rot_y.text),
                ParseFloat(rot_z.text)
            )
        );
    }



    // -------- SETTERS --------
    public void SetMapPosition(Vector3 position)
    {
        pos_x.text = position.x.ToString();
        pos_y.text = position.y.ToString();
        pos_z.text = position.z.ToString();
    }

    public void SetMapRotation(Vector3 rotation)
    {
        rot_x.text = rotation.x.ToString();
        rot_y.text = rotation.y.ToString();
        rot_z.text = rotation.z.ToString();
    }

    // -------- HELPER --------
    float ParseFloat(string value)
    {
        float result;
        if (!float.TryParse(value, out result))
        {
            result = 0f;
        }
        return result;
    }
}