"use client" //use react hook

import React, { useState } from "react"

const EmergencyStop = () => {

    const [isStopped, setIsStopped] = useState<boolean>(false)

    return (
        <div className="w-[320px] rounded-2xl bg-gray-800 p-5 shadow-lg border-red">
            {/* Emergency Stop Button */}
            <button onClick={() => setIsStopped(!isStopped)}
                className={`w-full rounded-xl text-lg font-bold 
                    ${isStopped ? "bg-blue-700 text-gray-300 hover:bg-blue-600" : "bg-red-600 text-white hover:bg-red-500"}`
                    }
                >
                {isStopped ? "Reset Emergency Stop" : "EMERGENCY STOP"}
            </button>
            
            {/* Footer */}
            <p className="mt-4 text-xs text-gray-400 text-center">Immediately disables all motors</p>
        </div>
    )
}

export default EmergencyStop