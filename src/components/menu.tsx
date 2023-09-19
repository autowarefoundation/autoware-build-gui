"use client"

import { useCallback } from "react"
import { WindowTitlebar } from "tauri-controls"

export function Menu() {
  const closeWindow = useCallback(async () => {
    const { appWindow } = await import("@tauri-apps/plugin-window")
    appWindow.close()
  }, [])

  return (
    <WindowTitlebar
      controlsOrder="right"
      // everything to the right of the window controls
      className="flex h-8 cursor-grab select-none items-center justify-end bg-background"
      windowControlsProps={{ platform: "windows", className: "w-fit" }}
    >
      <h1 className="pointer-events-none ml-auto font-sans text-lg font-semibold">
        Autoware Build GUI
      </h1>
    </WindowTitlebar>
  )
}
