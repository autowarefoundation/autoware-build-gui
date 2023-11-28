"use client";

import { useCallback, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { message } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";
import { Loader2 } from "lucide-react";
import { WindowTitlebar } from "tauri-controls";

import {
  autowareFolderPathAtom,
  buildLogsAtom,
  packageNamesAtom,
} from "@/app/jotai/atoms";

import { AboutDialog } from "./about-dialog";
import { Button } from "./ui/button";
import { Dialog, DialogContent, DialogTrigger } from "./ui/dialog";
import { toast } from "./ui/use-toast";

export function Menu() {
  const closeWindow = useCallback(async () => {
    const { appWindow } = await import("@tauri-apps/plugin-window");
    appWindow.close();
  }, []);

  const [autowarePath, _setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [_packages, setPackages] = useAtom(packageNamesAtom);
  const [_buildLogs, setBuildLogs] = useAtom(buildLogsAtom);
  const [updatingWorkspace, setUpdatingWorkspace] = useState(false);

  return (
    <WindowTitlebar
      controlsOrder="right"
      // everything to the right of the window controls
      className="flex h-10 cursor-grab select-none items-center justify-end bg-background pl-4"
      windowControlsProps={{ platform: "windows", className: "w-fit" }}
    >
      <Dialog modal={false}>
        <DialogTrigger asChild>
          <Button variant="ghost">About</Button>
        </DialogTrigger>
        <AboutDialog />
      </Dialog>

      <Button
        disabled={!autowarePath}
        onClick={async () => {
          // This will be handled by the backend and will trigger an update to the autoware workspace
          // git pull // vcs import // vcs pull // rosdep install // colcon build
          setUpdatingWorkspace(true);

          const payload = {
            path: autowarePath,
          };

          await invoke("update_autoware_workspace", {
            payload,
          });

          const packages = (
            (await invoke("get_package_names", {
              payload,
            })) as string[]
          ).map((name) => ({ name, status: false }));
          if (packages.length === 0) {
            await message("No packages found");
          }
          setPackages(packages);
          setBuildLogs([]);

          setUpdatingWorkspace(false);
          toast({
            title: "Workspace Updated",
            description: "Workspace has been updated successfully",
          });
        }}
        variant="ghost"
      >
        Update Workspace
      </Button>

      <Dialog modal open={updatingWorkspace}>
        <DialogContent>
          <div className="flex flex-col items-center justify-center">
            <Loader2 className="h-16 w-16 animate-spin" />
            <div className="mt-4 font-mono text-xl font-semibold">
              Updating Workspace
            </div>
          </div>
        </DialogContent>
      </Dialog>

      <Button
        variant="ghost"
        onClick={async () => {
          const res = await invoke("get_and_build_calibration_tools", {
            payload: { path: autowarePath },
          });

          if (res) {
            console.log("Calibration tools added successfully");
            toast({
              title: "Workspace Updated",
              description:
                "Workspace has been updated successfully with calibration tools",
            });
          } else {
            toast({
              title: "Failed to add calibration tools",
              description:
                "Failed to add calibration tools to the workspace. Please check the logs",
              variant: "destructive",
            });
            console.log("Failed to add calibration tools");
          }
        }}
      >
        Add Calibration Tools
      </Button>

      <h1 className="pointer-events-none ml-auto font-sans text-lg font-semibold">
        Autoware Build GUI
      </h1>
    </WindowTitlebar>
  );
}
