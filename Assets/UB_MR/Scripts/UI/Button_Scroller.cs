using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using UnityEngine.EventSystems;

namespace CAVAS.UB_MR.UI
{
    public abstract class Button_Scroller : MonoBehaviour
    {
        [SerializeField] GameObject buttonPrefab;
        [SerializeField] Transform viewport_content;
        float current = 0;
        List<RectTransform> all_elements;

        void Awake()
        {
            all_elements = new List<RectTransform>();
        }

        public void AddElement(string inName)
        {
            RectTransform newElement = GameObject.Instantiate(buttonPrefab, viewport_content).GetComponent<RectTransform>();

            Vector2 anchor = new Vector2(0.5f, 1); // Top Center Anchor
            newElement.anchorMin = anchor;
            newElement.anchorMax = anchor;
            newElement.pivot = anchor;
            newElement.anchoredPosition = new Vector2(0, current - newElement.rect.height);

            if (newElement.TryGetComponent<Button>(out Button button))
            {
                TextMeshProUGUI tmp = button.GetComponentInChildren<TextMeshProUGUI>();
                tmp.text = inName;
                button.onClick.AddListener(OnButtonClick);
            }
                

            all_elements.Add(newElement);
            current -= newElement.rect.height;
        }

        public void RemoveElement(RectTransform elem)
        {
            if (TryGetComponent<Button>(out Button button))
                button.onClick.RemoveListener(OnButtonClick);

            all_elements.Remove(elem);
            Destroy(elem.gameObject);
        }

        public void ArrangeElements()
        {

        }

        public void RemoveAllElements()
        {
            foreach (RectTransform rt in all_elements)
                RemoveElement(rt);
            all_elements.Clear();
        }
        
        public Button TryGetSelectedButton()
        {
            foreach (RectTransform rt in all_elements)
                if (rt.TryGetComponent<Button>(out Button button))
                    if (EventSystem.current.currentSelectedGameObject == button.gameObject)
                        return button;
            return null;
        }

        protected abstract void OnButtonClick();

    }
}
