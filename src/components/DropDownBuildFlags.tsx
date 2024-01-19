"use client";

import { useMemo } from "react";
import { useAtomValue } from "jotai";
import { ChevronsUpDown } from "lucide-react";

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { colconFlags, editedFlagsAtom } from "@/app/jotai/atoms";

import { BuildFlagsDialog } from "./BuildFlagsDialog";
import { Button } from "./ui/button";

export function DropDownBuildFlags() {
  const editedFlags = useAtomValue(editedFlagsAtom);
  const amountOfEditedFlags = Object.keys(editedFlags).length;

  const sortedColconFlags = useMemo(() => {
    return [...colconFlags] // Create a shallow copy to avoid mutating the original array
      .sort((a, b) => a.label.localeCompare(b.label)) // Alphabetical sort
      .sort((a, b) => {
        const aExistsInEditedFlags = a.label in editedFlags;
        const bExistsInEditedFlags = b.label in editedFlags;

        if (aExistsInEditedFlags && !bExistsInEditedFlags) {
          return -1; // a comes before b
        } else if (!aExistsInEditedFlags && bExistsInEditedFlags) {
          return 1; // b comes before a
        } else {
          return 0; // no change in order
        }
      });
  }, [colconFlags, editedFlags]);

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild className="flex w-full gap-2">
        <Button variant="outline" className="w-full justify-between capitalize">
          {`Build Flags Added - ${amountOfEditedFlags}`}
          <ChevronsUpDown className="ml-2 h-4 w-4 shrink-0 opacity-50" />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent
        style={{
          maxHeight: "16rem",
          overflow: "auto",
          width: "fit-content",
          marginRight: "1rem",
          marginTop: "0.5rem",
        }}
      >
        {sortedColconFlags.map((flag) => (
          <div
            key={flag.label}
            className="flex items-center justify-between rounded-md p-2 hover:bg-secondary"
          >
            {flag.label}
            <BuildFlagsDialog flag={flag} />
          </div>
        ))}
      </DropdownMenuContent>
    </DropdownMenu>
  );
}
