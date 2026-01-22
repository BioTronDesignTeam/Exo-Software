"use client" //react hook
import React, {useState} from "react"
import { AiFillSetting, AiFillControl } from "react-icons/ai";
import Dropdown from './Dropdown'


const ParameterTuning = () => {
    return (
        <div className="w-[320px] rounded-2xl bg-gray-800 p-5 shadow-lg border-red">
            {/* Header */}
            <div className="flex items-center justify-between mb-4">
                <h2 className="text-xl font-bold text-white">Parameter Tuning</h2>
            </div>
            <div className="border-b border-gray-600 mb-4" />

            {/* Dropdown menus */}
            <Dropdown title="PID Parameters">
                <Dropdown title="Hip">
                    <PID/>
                </Dropdown>
                <Dropdown title="Knee">
                    <PID/>
                </Dropdown>
            </Dropdown>
            <Dropdown title="Output Limits" >
                <OutputLimit />
            </Dropdown>
        </div>
    )
}

const PID = () => {
    const [Kp, setKp] = useState("")
    const [Ki, setKi] = useState("")
    const [Kd, setKd] = useState("")

    const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
        e.preventDefault();

        const Kp_final = Number(Kp)
        const Ki_final = Number(Ki)
        const Kd_final = Number(Kd)
        alert(`PID applied!`)
    };
    return( 

        <form className="flex-col gap-5" onSubmit={handleSubmit} >
            <label>Kp (Proportional)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={Kp} onChange={(e) => setKp(e.target.value)}/>
            </label><br/>
            <label>Ki (Integral)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={Ki} onChange={(e) => setKi(e.target.value)}/>
            </label><br/>
            <label>Kd (Derivative)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={Kd} onChange={(e) => setKd(e.target.value)}/>
            </label>
            <button className="w-[100%] text-center border-2 border-white rounded-s" type="submit">Apply PID</button>
        </form>
    )
}

const OutputLimit = () => {
    const[maxCurr, setMaxCurr] = useState("")
    const[maxVel, setMaxVel] = useState("")
    const[maxTor, setMaxTor] = useState("")
    
    const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
        e.preventDefault();

        const maxCurr_final = Number(maxCurr)
        const maxVel_final = Number(maxVel)
        const maxTor_final = Number(maxTor)
        alert(`Output limits applied!`)
    };

    return (
        <form className="flex-col gap-5" onSubmit={handleSubmit} >
            <label>Max Current (A)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={maxCurr} onChange={(e) => setMaxCurr(e.target.value)}/>
            </label><br/>
            <label>Max Velocity (*/s)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={maxVel} onChange={(e) => setMaxVel(e.target.value)}/>
            </label><br/>
            <label>Max Torque (Nm)
                <input className="bg-white text-black p-1 rounded-s" type="number" value={maxTor} onChange={(e) => setMaxTor(e.target.value)}/>
            </label>
            <button className="w-[100%] text-center border-2 border-white rounded-s" type="submit">Apply Output Limits</button>
        </form>        
    )
}

export default ParameterTuning