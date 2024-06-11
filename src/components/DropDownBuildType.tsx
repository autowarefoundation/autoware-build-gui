"use client";

import * as React from "react";
import { useAtom } from "jotai";
import { Check, ChevronsUpDown } from "lucide-react";

import { cn } from "@/lib/utils";
import { Button } from "@/components/ui/button";
import {
  Command,
  CommandEmpty,
  CommandGroup,
  CommandInput,
  CommandItem,
} from "@/components/ui/command";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import { colconBuildTypeAtom } from "@/app/jotai/atoms";

const buildTypes = [
  {
    value: "debug",
    label: "Debug",
  },
  {
    value: "release",
    label: "Release",
  },
  {
    value: "relwithdebinfo",
    label: "Release with Debug Info",
  },
];

export function DropDownBuildType() {
  const [open, setOpen] = React.useState(false);
  const [selected, setSelected] = useAtom(colconBuildTypeAtom);

  return (
    <Popover open={open} onOpenChange={setOpen}>
      <PopoverTrigger asChild>
        <Button
          variant="outline"
          role="combobox"
          aria-expanded={open}
          className="w-full justify-between"
        >
          <div className="w-[75%] truncate text-start text-xs capitalize">
            {selected.label !== "" ? selected.label : "Select build type"}
          </div>
          <ChevronsUpDown className="h-4 w-4 opacity-50" />
        </Button>
      </PopoverTrigger>
      <PopoverContent className="mr-12 mt-2 w-[200px] p-0">
        <Command>
          <CommandInput placeholder="Search build type" />
          <CommandEmpty>No build type found.</CommandEmpty>
          <CommandGroup>
            {buildTypes.map((buildType) => (
              <CommandItem
                key={buildType.value}
                onSelect={() => {
                  setSelected(buildType);
                  setOpen(false);
                }}
              >
                <Check
                  className={cn(
                    "mr-2 h-4 w-4",
                    selected === buildType ? "opacity-100" : "opacity-0"
                  )}
                />
                {buildType.label}
              </CommandItem>
            ))}
          </CommandGroup>
        </Command>
      </PopoverContent>
    </Popover>
  );
}
