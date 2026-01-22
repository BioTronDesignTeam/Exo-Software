import { createContext } from "react"
import { ReactNode } from "react"
import { useReducer } from "react"
export type PageState = {
    exampleField1: number
    exampleField2: number
    exampleField3: number
}

type Update = { key: keyof PageState, value: number };

//how to use:
export function pageStateReducer(state: PageState, action: Update): PageState {
    return {
        ...state,
        [action.key]: action.value
    }
}

export const PageStateContext = createContext<PageState | null>(null)
export const DispatchContext = createContext<React.Dispatch<Update> | null>(null);

export function PageStateProvider({ children }: { children: ReactNode }) {
    const emptyPageState: PageState = { exampleField1: 0, exampleField2: 0, exampleField3: 0 }

    const [state, dispatch] = useReducer(pageStateReducer, emptyPageState)

    return (
        <PageStateContext.Provider value={state}>
        <DispatchContext.Provider value={dispatch}>
            {children}
        </DispatchContext.Provider>
        </PageStateContext.Provider>
    )
}
