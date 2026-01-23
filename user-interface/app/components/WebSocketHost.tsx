"use client"

import { useEffect } from "react"
import { connectWebSocket } from "@/backend/connectWebSocket"

export function WebSocketHost(){
    useEffect(() => {
        const socket = connectWebSocket()
        return () => socket.close()
    }, [])

    return null
}