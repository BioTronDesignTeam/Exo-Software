'use client'

import { useEffect } from "react"
import { pageStateStore } from "../pageStateStore"
import { ActionType, usePageDispatch } from "../PageState"


export function PageUpdateDriver() {
  const dispatch = usePageDispatch()

  useEffect(() => {
    const id = setInterval(() => {
      dispatch({
        action_type: ActionType.UPDATE_PAGE,
        payload: {
          ...pageStateStore
        }
      })
    }, 1000 / 60)

    return () => clearInterval(id)
  }, [dispatch])

  return null
}
