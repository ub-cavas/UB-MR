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
                title = FindChild<TextMeshProUGUI>("Panel_Title (TMP)", "Title (TMP)");
            if (backButton is null)
                backButton = FindChild<Button>("Back_Button");
            
            this.gameObject.SetActive(true);
            if (backButton is not null)
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
            if (title is not null)
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

        private T FindChild<T>(params string[] childNames) where T : Component
        {
            foreach (string childName in childNames)
            {
                Transform child = transform.Find(childName);
                if (child is not null && child.TryGetComponent(out T component))
                    return component;
            }

            return null;
        }
    }
}
