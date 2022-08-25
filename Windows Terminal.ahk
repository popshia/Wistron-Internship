#NoEnv  ; Recommended for performance and compatibility with future AutoHotkey releases.
; #Warn  ; Enable warnings to assist with detecting common errors.
SendMode Input  ; Recommended for new scripts due to its superior speed and reliability.
SetWorkingDir %A_ScriptDir%  ; Ensures a consistent starting directory.

!r::ToggleTerminal()

ToggleTerminal()
{
    WinMatcher := "ahk_class CASCADIA_HOSTING_WINDOW_CLASS"

    DetectHiddenWindows, On

    if WinExist(WinMatcher)
    ; Window Exists
    {
        DetectHiddenWindows, Off

        ; Check if its hidden
        if !WinExist(WinMatcher) || !WinActive(WinMatcher)
        {
            WinShow ahk_class CASCADIA_HOSTING_WINDOW_CLASS
    	    WinActivate ahk_class CASCADIA_HOSTING_WINDOW_CLASS
        }
        else if WinExist(WinMatcher)
        {
            ; Script sees it without detecting hidden windows, so..
            WinHide ahk_class CASCADIA_HOSTING_WINDOW_CLASS
            Send !{Esc}
        }
    }
    else
    {
        Run "c:\Users\11107014\AppData\Local\Microsoft\WindowsApps\wt.exe"
        Sleep, 1000
        WinShow ahk_class CASCADIA_HOSTING_WINDOW_CLASS
    	WinActivate ahk_class CASCADIA_HOSTING_WINDOW_CLASS
    }
}