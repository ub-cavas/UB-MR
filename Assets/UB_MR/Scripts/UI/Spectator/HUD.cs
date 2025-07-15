using UnityEngine;
using CAVAS.UB_MR.DT;
using TMPro;

namespace CAVAS.UB_MR.UI.Spectator
{

    public class HUD : MonoBehaviour
    {
        [SerializeField] TextMeshProUGUI mSpectatingLabel;
        UB_MR.DT.Spectator spectator;

        void Awake()
        {
            spectator = GetComponentInParent<UB_MR.DT.Spectator>();
        }

        public void NextAgent()
        {
            Observable agent = spectator.Next();
            if (agent != null)
                this.mSpectatingLabel.text = ((MonoBehaviour)agent).gameObject.name;
            else
                this.mSpectatingLabel.text = "???";
        }

        public void PreviousAgent()
        {
            Observable agent = spectator.Previous();
            if (agent != null)
                this.mSpectatingLabel.text = ((MonoBehaviour)agent).gameObject.name;
            else
                this.mSpectatingLabel.text = "???";
        }
    }
}
