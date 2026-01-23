"use client"

import { createContext } from "react"
import { ReactNode } from "react"
import { useReducer } from "react"
import { useContext } from "react"
import { pageStateStore } from "./pageStateStore"


export type PageState = {
    battery_current: number
    battery_voltage: number
    exampleField1: number
}

export enum ActionType {
    UPDATE_PAGE = "UPDATE_PAGE",
    SAMPLE_BATTERY = "SAMPLE_BATTERY",
    SET_BATTERY = "SET_BATTERY",
    RANDOM_ACTION = "RANDOM_EXAMPLE"
}

type Action = 
    | { action_type: ActionType.UPDATE_PAGE; payload: Partial<PageState>}
    | { action_type: ActionType.SAMPLE_BATTERY; payload: { battery_current: number, battery_voltage: number } }
    | { action_type: ActionType.RANDOM_ACTION; payload: { random_string: String, random_number: number } }
//how to use:
export function pageStateReducer(state: PageState, action: Action): PageState {
    switch (action.action_type) {
        case ActionType.UPDATE_PAGE:
            return {
                ...state,
                ...action.payload
            }
        case ActionType.SAMPLE_BATTERY:
            return{
                ...state,
                battery_current: action.payload.battery_current,
                battery_voltage: action.payload.battery_voltage
            }
        case ActionType.RANDOM_ACTION:
            return {
                ...state,
                exampleField1: action.payload.random_number
            }
        default:
            return state
    }

}

export const PageStateContext = createContext<PageState | null>(null)
export const DispatchContext = createContext<React.Dispatch<Action> | null>(null);

export function PageStateProvider({ children }: { children: ReactNode }) {
    const emptyPageState: PageState = { 
        battery_current: 0,
        battery_voltage: 0,
        exampleField1: 0
    }

    const [state, dispatch] = useReducer(pageStateReducer, emptyPageState)

    return (
        <PageStateContext.Provider value={state}>
        <DispatchContext.Provider value={dispatch}>
            {children}
        </DispatchContext.Provider>
        </PageStateContext.Provider>
    )
}


export function usePageState() {
  const state = useContext(PageStateContext)
  if (!state) throw new Error("usePageState must be used inside a PageStateProvider")
  return state
}

export function usePageDispatch() {
  const dispatch = useContext(DispatchContext)
  if (!dispatch) throw new Error("usePageDispatch must be used inside a PageStateProvider")
  return dispatch
}