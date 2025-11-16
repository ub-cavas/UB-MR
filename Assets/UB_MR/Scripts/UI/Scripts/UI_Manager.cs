using CAVAS.UI;
using UnityEngine;

public static class UI_Manager
{
    static Panel prevPanel;
    static Panel curPanel;

    public static void LoadPreviousPanel()
    {
        if (prevPanel is not null)
            LoadPanel(prevPanel);
    }

    public static void LoadPanel(Panel inPanel)
    {
        prevPanel = curPanel;
        curPanel = inPanel;
        if (prevPanel && prevPanel != curPanel)
            prevPanel.UnloadPanel();
        curPanel.LoadPanel();
    }

    public static void UnloadPanel(Panel inPanel)
    {
        curPanel.UnloadPanel();
    }

    public static void SoftUpdate(Panel inPanel)
    {
        if (prevPanel is null)
        {
            prevPanel = inPanel;
            curPanel = inPanel;
            LoadPanel(inPanel);
        }
            
    }
}
