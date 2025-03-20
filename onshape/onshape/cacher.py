"""Defines a utility function for caching requests."""

import datetime
import logging
from pathlib import Path
from typing import Any, Callable, Coroutine, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")


class Cacher:
    def __init__(self, cache_dir: str | Path, invalidate_after_n_minutes: float | None = None) -> None:
        self.cache_map: dict[str, Any] = {}
        self.cache_dir = Path(cache_dir).expanduser().resolve()
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.invalidate_after_n_minutes = invalidate_after_n_minutes

    def _run_fn(
        self,
        file_path: Path,
        invalidate_after_n_minutes: float | None = None,
    ) -> bool:
        if not file_path.exists():
            return True
        if file_path.stat().st_size == 0:
            return True
        if invalidate_after_n_minutes is None:
            invalidate_after_n_minutes = self.invalidate_after_n_minutes
        if invalidate_after_n_minutes is None:
            return False
        modified_time = datetime.datetime.fromtimestamp(file_path.stat().st_mtime)
        rerun_fn = (datetime.datetime.now() - modified_time).seconds > invalidate_after_n_minutes * 60
        if rerun_fn:
            logger.warning("Cache invalidated after %.2f minutes", invalidate_after_n_minutes)
            file_path.unlink()
        return rerun_fn

    async def __call__(
        self,
        cache_key: str,
        get_fn: Callable[[], Coroutine[Any, Any, T]],
        from_json_fn: Callable[[str], T],
        to_json_fn: Callable[[T], str],
        invalidate_after_n_minutes: float | None = None,
    ) -> T:
        """Gets an item by calling an API or by retrieving it from the cache.

        Args:
            cache_dir: The directory to store the cache.
            cache_key: The key to use for the cache.
            get_fn: The function to call to get the item.
            from_json_fn: The function to call to parse the item from JSON.
            to_json_fn: The function to call to serialize the item to JSON.
            invalidate_after_n_minutes: The number of minutes after which the cache should be invalidated.

        Returns:
            The item.
        """
        if cache_key in self.cache_map:
            return self.cache_map[cache_key]
        cache_path = self.cache_dir / f"{cache_key}.json"
        if self._run_fn(cache_path, invalidate_after_n_minutes):
            item = await get_fn()
            with open(cache_path, "w") as f:
                f.write(to_json_fn(item))
            self.cache_map[cache_key] = item
        else:
            with open(cache_path) as f:
                item = from_json_fn(f.read())
            self.cache_map[cache_key] = item
        return item
