using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UI
{
    public abstract class Panel : MonoBehaviour
    {
        TextMeshProUGUI title;
        Button backButton;
        RectTransform[] allElements;

        void Awake()
        {
            UI_Manager.SoftUpdate(this);
        }

        public virtual void LoadPanel()
        {
            if (title is null)
                title = transform.Find("Panel_Title (TMP)").GetComponent<TextMeshProUGUI>();
            if (backButton is null)
                backButton = transform.Find("Back_Button").GetComponent<Button>();
            
            this.gameObject.SetActive(true);
            backButton.onClick.AddListener(Back);
        }

        public virtual void UnloadPanel()
        {
            if (backButton is not null)
                backButton.onClick.RemoveAllListeners();
                
            this.gameObject.SetActive(false);
        }

        protected void SetTitle(string inTitle)
        {
            title.text = inTitle;
        }

        protected Button GetBackButton()
        {
            return backButton;
        }
        
        protected void Back()
        {
            UI_Manager.LoadPreviousPanel();
        }
    }
}
