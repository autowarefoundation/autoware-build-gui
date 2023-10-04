"use client"

import "@/styles/globals.css"

import { Provider } from "jotai"

import { cn } from "@/lib/utils"
import { Menu } from "@/components/menu"
import { StyleSwitcher } from "@/components/style-switcher"
import { ThemeProvider } from "@/components/theme-provider"

interface ExamplesLayoutProps {
  children: React.ReactNode
}

export default function MyApp({ children }: ExamplesLayoutProps) {
  return (
    <html lang="en" suppressHydrationWarning className="overflow-clip bg-black">
      <head />
      <body className="overflow-clip bg-transparent font-sans antialiased scrollbar-none">
        <Provider>
          <ThemeProvider attribute="class" defaultTheme="system" enableSystem>
            <div className="min-h-screen overflow-clip">
              <Menu />
              <div
                className={cn(
                  "h-screen overflow-auto border-t bg-background pb-8",
                  // "scrollbar-none",
                  "scrollbar scrollbar-track-transparent scrollbar-thumb-accent scrollbar-thumb-rounded-md",
                  "select-none"
                )}
              >
                {children}
              </div>
            </div>
            {/* <TailwindIndicator /> */}
          </ThemeProvider>
          <StyleSwitcher />
        </Provider>
      </body>
    </html>
  )
}

// export const metadata: Metadata = {
//   icons: {
//     shortcut: ["#"],
//   },
// }
