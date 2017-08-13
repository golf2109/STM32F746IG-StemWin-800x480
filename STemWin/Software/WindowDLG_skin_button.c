#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0    (GUI_ID_USER + 0x00)
#define ID_BUTTON_0    (GUI_ID_USER + 0x01)
#define ID_BUTTON_1    (GUI_ID_USER + 0x02)
#define ID_BUTTON_2    (GUI_ID_USER + 0x03)
#define ID_BUTTON_3    (GUI_ID_USER + 0x04)

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 320, 240, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 20, 10, 80, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_1, 20, 40, 80, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_2, 20, 70, 80, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_3, 20, 100, 80, 20, 0, 0x0, 0 },
};

static BUTTON_SKINFLEX_PROPS _aProps[] = {
// BUTTON_SKINFLEX_PI_PRESSED
  {
    {GUI_RED, GUI_RED, GUI_DARKRED},
    {GUI_LIGHTRED, GUI_RED },
    {GUI_RED, GUI_DARKRED },
     3
  },
// BUTTON_SKINFLEX_PI_FOCUSSED
  {
    {GUI_DARKRED, GUI_RED, GUI_DARKGRAY},
    {GUI_LIGHTGRAY, GUI_GRAY },
    {GUI_GRAY, GUI_DARKGRAY },
     3
  },
// BUTTON_SKINFLEX_PI_ENABLED
  {
    {GUI_DARKRED, GUI_GRAY, GUI_DARKGRAY},
    {GUI_LIGHTGRAY, GUI_GRAY },
    {GUI_GRAY, GUI_DARKGRAY },
     3
  },
// BUTTON_SKINFLEX_PI_DISABLED
  {
    {0, 0, 0},
    {0, 0 },
    {0, 0 },
     3
  },
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _CustomSkin
*/
static int _CustomSkin(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo) {
  int       Id;
  GUI_POINT aPoints[3];
  GUI_COLOR Color;
  BUTTON_SKINFLEX_PROPS aPropsOld[4];
  int i;

  switch (pDrawItemInfo->Cmd) {
  case WIDGET_ITEM_DRAW_BACKGROUND:
    Id = WM_GetId(pDrawItemInfo->hWin);
    switch (Id) {
    case ID_BUTTON_0:
      //
      // Draw an elliptic button
      //
      if (BUTTON_IsPressed(pDrawItemInfo->hWin)) {
        Color = GUI_YELLOW;
      } else {
        Color = GUI_RED;
      }
      GUI_SetColor(Color);
      GUI_FillEllipse(pDrawItemInfo->x1 / 2, pDrawItemInfo->y1 / 2, pDrawItemInfo->x1 / 2 - 1, pDrawItemInfo->y1 / 2 - 1);
      break;
    case ID_BUTTON_1:
      //
      // Draw a triangle button
      //
      aPoints[0].x = pDrawItemInfo->x1 / 2;
      aPoints[0].y = pDrawItemInfo->y0;
      aPoints[1].x = pDrawItemInfo->x1;
      aPoints[1].y = pDrawItemInfo->y1;
      aPoints[2].x = pDrawItemInfo->x0;
      aPoints[2].y = pDrawItemInfo->y1;
      if (BUTTON_IsPressed(pDrawItemInfo->hWin)) {
        Color = GUI_GREEN;
      } else {
        Color = GUI_CYAN;
      }
      GUI_SetColor(Color);
      GUI_FillPolygon(aPoints, GUI_COUNTOF(aPoints), pDrawItemInfo->x0, pDrawItemInfo->y0);
      break;
    case ID_BUTTON_2:
      //
      // Draw a standard button with different colors
      //
      for (i = 0; i < 4; i++) {
        BUTTON_GetSkinFlexProps(&aPropsOld[i], i);  // Get default properties
        BUTTON_SetSkinFlexProps(&_aProps[i], i);    // Set new properties
      }
      BUTTON_DrawSkinFlex(pDrawItemInfo);           // Draw button with new properties
      for (i = 0; i < 4; i++) {
        BUTTON_SetSkinFlexProps(&aPropsOld[i], i);  // Restore old properties to avoid other buttons will be drawn with this properties
      }
      break;
    case ID_BUTTON_3:
      BUTTON_DrawSkinFlex(pDrawItemInfo);
      break;
    }
    return 0;
  default:
    return BUTTON_DrawSkinFlex(pDrawItemInfo);
  }
}

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetSkin(hItem, _CustomSkin);
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetSkin(hItem, _CustomSkin);
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetSkin(hItem, _CustomSkin);
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_3);
    BUTTON_SetSkin(hItem, _CustomSkin);
    break;
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

WM_HWIN CreateWindow(void);
WM_HWIN CreateWindow(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

/*********************************************************************
*
*       MainTask
*/
//////////void MainTask(void) {
//////////  //
//////////  // Set flag for automatic use of memory devices. Calling before GUI_Init() makes sure the desktop window uses them, too.
//////////  //
//////////  WM_SetCreateFlags(WM_CF_MEMDEV);
//////////  //
//////////  // Initialize GUI
//////////  //
//////////  GUI_Init();
//////////  GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
//////////  while (1) {
//////////    GUI_Delay(100);
//////////  }
//////////}

