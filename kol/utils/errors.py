"""Defines utility functions used by the project."""

from typing import Any, Coroutine, TypeVar

T = TypeVar("T")


async def catch_error(coro: Coroutine[Any, Any, T]) -> tuple[T | None, Exception | None]:
    try:
        return await coro, None
    except Exception as e:
        return None, e
