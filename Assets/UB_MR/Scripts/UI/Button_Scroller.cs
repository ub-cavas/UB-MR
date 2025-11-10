using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    public abstract class Button_Scroller : MonoBehaviour
    {
        [SerializeField] GameObject buttonPrefab;
        [SerializeField] Transform viewport_content;
        float current = 0;
        List<RectTransform> all_elements;

        protected virtual void Start()
        {
            all_elements = new List<RectTransform>();
            // Start adding buttons with an offset from the top
            //current = -buttonPrefab.GetComponent<RectTransform>().rect.height;
        }

        //TODO: Insert a new text element to the scroll view
        public void AddElement()
        {
            RectTransform newElement = GameObject.Instantiate(buttonPrefab, viewport_content).GetComponent<RectTransform>();

            Vector2 anchor = new Vector2(0.5f, 1); // Top Center Anchor
            newElement.anchorMin = anchor;
            newElement.anchorMax = anchor;
            newElement.pivot = anchor;
            newElement.anchoredPosition = new Vector2(0, current - newElement.rect.height);

            if (newElement.TryGetComponent<Button>(out Button button))
                button.onClick.AddListener(OnButtonClick);

            all_elements.Add(newElement);
            current -= newElement.rect.height;
        }

        public void RemoveElement(RectTransform elem)
        {
            if (TryGetComponent<Button>(out Button button))
                button.onClick.RemoveListener(OnButtonClick);

            all_elements.Remove(elem);
        }

        protected abstract void OnButtonClick();

    }
}
