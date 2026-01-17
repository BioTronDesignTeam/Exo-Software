"use client" // add this to use react hook

import React, { useState } from "react"

const BatteryStatus = () => {
  const [voltage, setVoltage] = useState<number>(12.6)
  const [current, setCurrent] = useState<number>(1.8)

  // TODO: setup websocket communication

  /*
  useEffect(() => {
    const ws = new WebSocket("ws://YOUR_FIRMWARE_IP:PORT")

    ws.onopen = () => {
      console.log("WebSocket connected")
    }

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        // update voltage and current values. 
        // The recieved data from firmwareformat must be { voltage: number, current: number }
        setVoltage(data.voltage)
        setCurrent(data.current)
      } catch (err) {
        console.error("Invalid WebSocket message", err)
      }
    }

    ws.onerror = (err) => {
      console.error("WebSocket error", err)
    }

    ws.onclose = () => {
      console.log("WebSocket disconnected")
    }

    // Cleanup on component unmount
    return () => {
      ws.close()
    }
  }, []) // empty dependency = run once
  */

  return (
    <div className="w-[320px] rounded-2xl bg-gray-800 p-5 shadow-lg">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-bold text-white">Battery Status</h2>
        <span className="h-3 w-3 rounded-full bg-green-400" />
      </div>

      <div className="border-b border-gray-600 mb-4" />

      {/* Metrics */}
      <div className="grid grid-cols-2 gap-4">
        <div className="rounded-xl bg-gray-700 p-4 text-center">
          <p className="text-sm text-gray-300">Voltage</p>
          <p className="text-2xl font-semibold text-white">
            {voltage}
            <span className="text-sm font-normal text-gray-300"> V</span>
          </p>
        </div>

        <div className="rounded-xl bg-gray-700 p-4 text-center">
          <p className="text-sm text-gray-300">Current</p>
          <p className="text-2xl font-semibold text-white">
            {current}
            <span className="text-sm font-normal text-gray-300"> A</span>
          </p>
        </div>
      </div>

      {/* Optional footer */}
      <p className="mt-4 text-xs text-gray-400 text-center">
        Live telemetry
      </p>
    </div>
  )
}

export default BatteryStatus
