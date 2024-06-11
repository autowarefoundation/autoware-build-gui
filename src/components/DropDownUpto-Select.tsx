"use client";

import { useRef, useState } from "react";
import { useAtom } from "jotai";
import { ChevronsUpDown } from "lucide-react";

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { buildTypeAtom } from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import { Label } from "./ui/label";
import { Separator } from "./ui/separator";

export function DropDownUptoSelect() {
  const [buildType, setBuildType] = useAtom(buildTypeAtom);
  const triggerRef = useRef<HTMLButtonElement>(null);
  const [open, setOpen] = useState(false);

  return (
    <DropdownMenu open={open} onOpenChange={setOpen}>
      <DropdownMenuTrigger asChild className="flex w-full gap-2">
        <Button
          ref={triggerRef}
          onClick={() => setOpen(!open)}
          variant="outline"
          className="w-full justify-between text-xs uppercase"
        >
          {buildType}
          <ChevronsUpDown className="ml-2 h-4 w-4 shrink-0 opacity-50" />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent className="space-y-2">
        <Label className="p-2">Colcon Build Packages</Label>
        <Separator />
        <div className="flex flex-col">
          {/* upto or select */}
          <Label
            onClick={() => {
              setOpen(!open);
              setBuildType("up-to");
            }}
            className="cursor-pointer rounded-md p-2 text-xs capitalize hover:bg-muted"
          >
            UP_TO
          </Label>
          <Label
            onClick={() => {
              setOpen(!open);
              setBuildType("select");
            }}
            className="cursor-pointer rounded-md p-2 text-xs capitalize hover:bg-muted"
          >
            SELECT
          </Label>

          <Label
            onClick={() => {
              setOpen(!open);
              setBuildType("above");
            }}
            className="cursor-pointer rounded-md p-2 text-xs capitalize hover:bg-muted"
          >
            ABOVE
          </Label>
          <Label
            onClick={() => {
              setOpen(!open);
              setBuildType("skip");
            }}
            className="cursor-pointer rounded-md p-2 text-xs capitalize hover:bg-muted"
          >
            SKIP
          </Label>
        </div>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}
