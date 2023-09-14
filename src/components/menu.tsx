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
      className="ml-20 flex h-8 items-center bg-background text-white"
      windowControlsProps={{ platform: "windows", justify: true }}
    >
      <h1 className="ml-auto select-none font-sans text-lg font-semibold">
        Autoware Build GUI
      </h1>
    </WindowTitlebar>
  )
}
