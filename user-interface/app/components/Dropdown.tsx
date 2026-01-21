"use client"
import React, { useState } from "react"

type DropdownProps = {
  title: string
  icon?: React.ReactNode
  children: React.ReactNode
}

const Dropdown = ({ title, icon, children }: DropdownProps) => {
  const [open, setOpen] = useState(false)

  return (
    <div className="w-[320px] bg-gray-800 p-4 shadow-lg">
      {/* Header */}
      <button
        onClick={() => setOpen(!open)}
        className="w-full flex items-center justify-between text-left text-white font-semibold"
      >
        <div className="flex items-center gap-3">
          {icon && <span className="text-gray-300">{icon}</span>}
          <span>{title}</span>
        </div>

        <span className="text-gray-400">
          {open ? "▾" : "▸"}
        </span>
      </button>

      {/* Content */}
      {open && (
        <div className="rounded-xl p-2 text-sm text-gray-300">
          {children}
        </div>
      )}
    </div>
  )
}

export default Dropdown