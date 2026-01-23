"use client"

import { usePageState } from "../PageState"
const DummyWebsocketDisplay = () => {
    const state = usePageState()

    return (
        <div>
            <p>Dummy Data (Random): {state.exampleField1}</p>
            <p>Battery Voltage(Fake): {state.battery_voltage}</p>
            <p>Battery Current(Fake): {state.battery_current}</p>
        </div>
    )
}

export default DummyWebsocketDisplay