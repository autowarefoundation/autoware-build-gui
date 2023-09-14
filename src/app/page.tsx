"use client"

import { useEffect } from "react"
import { invoke } from "@tauri-apps/api/tauri"
import { message } from "@tauri-apps/plugin-dialog"
import { useAtom } from "jotai"

import LeftPane from "@/components/leftPane"
import RightPane from "@/components/rightPane"
import {
  autowareFolderPathAtom,
  buildLogsAtom,
  packageNamesAtom,
} from "@/app/jotai/atoms"

export default function App() {
  const [packages, setPackages] = useAtom(packageNamesAtom)
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  )
  const [buildLogs, setBuildLogs] = useAtom(buildLogsAtom)

  useEffect(() => {
    if (!autowareFolderPath) return
    async function getPackages() {
      const payload = {
        path: autowareFolderPath,
      }
      const packages = (
        (await invoke("get_package_names", {
          payload,
        })) as string[]
      ).map((name) => ({ name, status: false }))
      if (packages.length === 0) {
        await message("No packages found")
      }
      setPackages(packages)
      setBuildLogs([])
    }
    getPackages()
  }, [autowareFolderPath])

  return (
    <div className="flex h-full flex-row justify-between">
      <LeftPane />
      <RightPane />
    </div>
  )
}
