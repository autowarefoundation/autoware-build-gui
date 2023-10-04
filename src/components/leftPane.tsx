"use client";

import React, { useState } from "react";
import { useAtom, useAtomValue } from "jotai";

import { cn } from "@/lib/utils";
import {
  minimalSetupAWSimAtom,
  minimalSetupLoggingSimulatorAtom,
  minimalSetupPlanningSimulatorAtom,
  packageNamesAtom,
} from "@/app/jotai/atoms";

import { Search } from "./SearchBar";
import { Button } from "./ui/button";
import { Checkbox } from "./ui/checkbox";
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from "./ui/select";

const LeftPane = () => {
  const [packages, setPackages] = useAtom(packageNamesAtom);
  const minimalPlanningSetup = useAtomValue(minimalSetupPlanningSimulatorAtom);
  const minimalSimulatorSetup = useAtomValue(minimalSetupLoggingSimulatorAtom);
  const minimalAWSimulatorSetup = useAtomValue(minimalSetupAWSimAtom);

  const [search, setSearch] = React.useState("");

  const filteredPackages = packages
    .sort((a, b) => a.name.localeCompare(b.name))
    .filter((packageItem) => {
      if (search === "") {
        return true; // if search is empty, include all packages
      }
      return packageItem.name.toLowerCase().includes(search.toLowerCase());
    });

  const setAllPackages = () => {
    const newPackages = [...reFilteredPackages];

    const allPackagesSelected = newPackages.every(
      (packageItem) => packageItem.status
    );

    newPackages.forEach((packageItem) => {
      packageItem.status = !allPackagesSelected;
    });
    // only set the packages that are visible without changing the status of the hidden packages
    const newAllPackages = [...packages];
    newPackages.forEach((packageItem) => {
      const packageIndex = newAllPackages.findIndex(
        (item) => item.name === packageItem.name
      );
      newAllPackages[packageIndex].status = packageItem.status;
    });
    setPackages(newAllPackages);
  };

  const [selectedMinimalSetup, setSelectedMinimalSetup] = useState<
    "PlanningSim" | "LoggingSim" | "None" | "AWSim"
  >("None");

  const reFilteredPackages =
    selectedMinimalSetup === "PlanningSim"
      ? filteredPackages.filter((packageItem) =>
          minimalPlanningSetup.includes(packageItem.name)
        )
      : selectedMinimalSetup === "LoggingSim"
      ? filteredPackages.filter((packageItem) =>
          minimalSimulatorSetup.includes(packageItem.name)
        )
      : selectedMinimalSetup === "AWSim"
      ? filteredPackages.filter((packageItem) =>
          minimalAWSimulatorSetup.includes(packageItem.name)
        )
      : filteredPackages;

  return (
    <div className="flex w-full flex-col gap-4 p-4">
      <div className="flex items-center gap-4">
        <Search search={search} setSearch={setSearch} />
        <Button onClick={setAllPackages} variant="secondary">
          {reFilteredPackages.every((packageItem) => packageItem.status)
            ? "Deselect all"
            : "Select all"}
        </Button>
      </div>
      <Select
        onValueChange={(value) => {
          switch (value) {
            case "PlanningSim":
              // go through the array and set the status of the packages
              // to true if they are in the minimal setup
              const newPackages = [...packages];
              newPackages.forEach((packageItem) => {
                if (minimalPlanningSetup.includes(packageItem.name)) {
                  packageItem.status = true;
                } else {
                  packageItem.status = false;
                }
              });
              setPackages(newPackages);
              setSelectedMinimalSetup("PlanningSim");
              setSearch("");

              break;
            case "LoggingSim":
              const newPackages2 = [...packages];
              newPackages2.forEach((packageItem) => {
                if (minimalSimulatorSetup.includes(packageItem.name)) {
                  packageItem.status = true;
                } else {
                  packageItem.status = false;
                }
              });
              setPackages(newPackages2);
              setSelectedMinimalSetup("LoggingSim");
              setSearch("");

              break;
            case "AWSim":
              const newPackages4 = [...packages];
              newPackages4.forEach((packageItem) => {
                if (minimalAWSimulatorSetup.includes(packageItem.name)) {
                  packageItem.status = true;
                } else {
                  packageItem.status = false;
                }
              });
              setPackages(newPackages4);
              setSelectedMinimalSetup("AWSim");
              setSearch("");

              break;
            case "None":
              const newPackages3 = [...packages];
              newPackages3.forEach((packageItem) => {
                packageItem.status = false;
              });
              setPackages(newPackages3);
              setSelectedMinimalSetup("None");
              setSearch("");
              break;

            default:
              break;
          }
        }}
        defaultValue="None"
        disabled={packages.length === 0}
      >
        <SelectTrigger className="w-[16rem]">
          <SelectValue placeholder="Select a minimal setup(optional)" />
        </SelectTrigger>
        <SelectContent>
          <SelectGroup>
            <SelectLabel>Minimal Setup</SelectLabel>
            <SelectItem value="None">None</SelectItem>
            <SelectItem value="PlanningSim">
              Planning Simulator Minimal Setup
            </SelectItem>
            <SelectItem value="LoggingSim">
              Logging Simulator Minimal Setup
            </SelectItem>
            <SelectItem value="AWSim">AWSim Minimal Setup</SelectItem>
          </SelectGroup>
        </SelectContent>
      </Select>
      <div
        className={cn(
          "flex min-w-fit max-w-[20rem] flex-col gap-2 overflow-y-auto", // scrollable div
          "h-[50rem] rounded-md border p-2",
          "select-text"
          // filteredPackages.length > 0 ? "rounded-md border p-2" : "opacity-0"
        )}
      >
        {reFilteredPackages.map((packageItem, idx) => (
          <div
            className="flex items-center justify-start space-x-2"
            key={`${packageItem.name}-${idx}`}
          >
            <Checkbox
              id={packageItem.name}
              checked={packageItem.status}
              onCheckedChange={(checked) => {
                const newPackages = [...packages];
                const packageIndex = newPackages.findIndex(
                  (item) => item.name === packageItem.name
                );
                newPackages[packageIndex].status = checked as boolean;

                setPackages(newPackages);
              }}
            />
            <label htmlFor={packageItem.name}>{packageItem.name}</label>
          </div>
        ))}
      </div>
    </div>
  );
};

export default LeftPane;
