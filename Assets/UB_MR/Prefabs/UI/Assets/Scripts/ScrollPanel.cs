using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

namespace CAVAS.UI
{
    public abstract class ScrollPanel : Panel
    {
        [SerializeField] Transform viewport_content;
        [SerializeField] protected GameObject buttonPrefab;
        [SerializeField] protected Button addButton;
        [SerializeField] protected Button editButton;
        [SerializeField] protected Button removeButton;

        List<Button> scroll_buttons;
        Button activeButton;

        float current = 0;

        public override void LoadPanel()
        {
            addButton.onClick.AddListener(OnAddClicked);
            editButton.onClick.AddListener(OnEditClicked);
            removeButton.onClick.AddListener(OnRemoveClicked);
            base.LoadPanel();
        }

        protected void AddButton(string inLabel)
        {
            if (scroll_buttons is null)
                scroll_buttons = new List<Button>();
            
            Button button = GameObject.Instantiate(buttonPrefab, viewport_content).GetComponent<Button>();
            RectTransform rt = button.GetComponent<RectTransform>();
            // Top Center Anchor
            Vector2 anchor = new Vector2(0.5f, 1); 
            rt.anchorMin = anchor;
            rt.anchorMax = anchor;
            rt.pivot = anchor;
            rt.anchoredPosition = new Vector2(0, current - rt.rect.height);
            // Set Label
            TextMeshProUGUI tmp = button.GetComponentInChildren<TextMeshProUGUI>();
                tmp.text = inLabel;
                button.onClick.AddListener(OnScrollButtonClick);
            // Add to scroll_buttons
            scroll_buttons.Add(button);
            current -= rt.rect.height;
        }
        
        public void RemoveButton(Button inButton)
        {
            inButton.onClick.RemoveAllListeners();
            Destroy(inButton.gameObject);
        }

        public void RemoveAllButtons()
        {
            for (int i = 0; i < scroll_buttons.Count; i++)
                RemoveButton(scroll_buttons[i]);
            scroll_buttons.Clear();
            current = 0;
        }
        
        public override void UnloadPanel()
        {
            RemoveAllButtons();
            base.UnloadPanel();
        }

        protected Button GetSelectedButton()
        {
            foreach (Button button in scroll_buttons)
                if (EventSystem.current.currentSelectedGameObject == button.gameObject)
                    return button;
            return null;
        }

        protected string GetSelectedButtonLabel()
        {
            Button button = GetSelectedButton();
            if (button)
                return button.GetComponentInChildren<TextMeshProUGUI>().text;
            return null;
        }

        protected abstract void OnAddClicked();
        protected abstract void OnRemoveClicked();
        protected abstract void OnEditClicked();

        Button GetActiveButton()
        {
            return activeButton;
        }
        
        string GetActiveButtonLabel()
        {
            return GetActiveButton().GetComponentInChildren<TextMeshProUGUI>().text;
        }

        void OnScrollButtonClick()
        {
            activeButton = GetSelectedButton();
        }
   
    }
}
