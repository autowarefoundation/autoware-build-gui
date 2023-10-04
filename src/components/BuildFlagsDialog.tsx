"use client"

import { useRef, useState } from "react"
import { message } from "@tauri-apps/plugin-dialog"
import { useAtom } from "jotai"

import { Button } from "@/components/ui/button"
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { editedFlagsAtom } from "@/app/jotai/atoms"

import { Checkbox } from "./ui/checkbox"

export function BuildFlagsDialog({
  flag,
}: {
  flag: {
    label: string
    placeholder: string
    multipleValues: boolean
  }
}) {
  const [editedFlags, setEditedFlags] = useAtom(editedFlagsAtom)
  const [inputValues, setInputValues] = useState<string[]>(
    editedFlags[flag.label] ? [editedFlags[flag.label]] : []
  )

  const closeRef = useRef<HTMLButtonElement>(null)

  const handleSave = () => {
    if (inputValues.some((value) => value.includes("DCMAKE_BUILD_TYPE"))) {
      // Display an error or warning to the user
      message(
        "Modifying DCMAKE_BUILD_TYPE is not allowed!, it is being ignored from the flag. Use the build type dropdown instead."
      )
      const newValues = [...inputValues]
      newValues.splice(
        inputValues.findIndex((value) => value.includes("DCMAKE_BUILD_TYPE")),
        1
      )

      const valueToSave = flag.placeholder
        ? newValues.join(" ")
        : newValues[0]
        ? "enabled"
        : ""

      setEditedFlags((prev) => ({ ...prev, [flag.label]: valueToSave }))
    } else {
      const valueToSave = flag.placeholder
        ? inputValues.join(" ")
        : inputValues[0]
        ? "enabled"
        : ""

      setEditedFlags((prev) => ({ ...prev, [flag.label]: valueToSave }))
    }
  }

  const handleRemove = () => {
    const updatedFlags = { ...editedFlags }
    delete updatedFlags[flag.label]
    setEditedFlags(updatedFlags)
  }
  const flagExists = Boolean(editedFlags[flag.label])

  return (
    <Dialog>
      <DialogTrigger ref={closeRef} asChild>
        <Button variant="outline">{flagExists ? "Update" : "Add"} Flag</Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>
            {flagExists ? "Update" : "Add"} {flag.label}
          </DialogTitle>
          <DialogDescription>{flag.placeholder}</DialogDescription>
        </DialogHeader>
        <div className="grid gap-4 py-4">
          <div className="flex flex-col items-center justify-center gap-4">
            <Label htmlFor="flagValue" className="text-right">
              {flag.placeholder && "Value"}
              {!flag.placeholder && "Set/Unset :"}
            </Label>
            {flag.multipleValues ? (
              <div className="col-span-3 flex w-full flex-col gap-2">
                {inputValues.map((value, index) => (
                  <div key={index} className="col-span-3 flex items-center">
                    <Input
                      value={value}
                      onChange={(e) => {
                        const newValues = [...inputValues]
                        newValues[index] = e.target.value
                        setInputValues(newValues)
                      }}
                    />
                    <Button
                      onClick={() => {
                        const newValues = [...inputValues]
                        newValues.splice(index, 1)
                        setInputValues(newValues)
                      }}
                    >
                      Remove
                    </Button>
                  </div>
                ))}
              </div>
            ) : !flag.placeholder ? (
              <div className="flex items-center gap-2">
                <Checkbox
                  id="flagValue"
                  checked={!!inputValues[0]}
                  onCheckedChange={(checked) =>
                    setInputValues([checked ? "enabled" : ""])
                  }
                  className="col-span-3"
                />
                <span>{inputValues[0] ? "Enabled" : "Disabled"}</span>
              </div>
            ) : (
              <Input
                id="flagValue"
                value={inputValues[0] || ""}
                onChange={(e) => setInputValues([e.target.value])}
                className="col-span-3"
              />
            )}
          </div>
          {flag.multipleValues && (
            <Button onClick={() => setInputValues([...inputValues, ""])}>
              {inputValues.length === 0 ? "Add" : "Add Another"}
            </Button>
          )}
        </div>
        <DialogFooter>
          <Button
            variant="destructive"
            onClick={() => {
              if (flagExists) handleRemove()
              closeRef.current?.click()
            }}
          >
            {flagExists ? "Remove" : "Close"}
          </Button>
          <Button
            onClick={() => {
              handleSave()
              closeRef.current?.click()
            }}
          >
            {flagExists ? "Update" : "Add"}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
