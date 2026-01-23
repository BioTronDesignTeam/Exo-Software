import { pageStateStore } from "../app/pageStateStore";

export function connectWebSocket() {
    const socket = new WebSocket("ws://localhost:8080")

    socket.onopen = () => {
        console.log("Socket connected")
    }

    socket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            pageStateStore.battery_current = data.battery_current
            pageStateStore.battery_voltage = data.battery_voltage

            pageStateStore.exampleField1 = data.exampleField1

            console.log("updated page state");
        } catch (e) {
            console.error("JSON from websocket formatting invalid");
        }
    }

    socket.onerror = (err) => {
        console.error("socket error", err)
    }

    socket.onclose = () => {
        console.log("socket closed")
    }
    return socket
}