"""Defines a cached version of the Onshape API."""

import asyncio
import hashlib
from typing import BinaryIO

from onshape.onshape.api import OnshapeApi
from onshape.onshape.cacher import Cacher
from onshape.onshape.client import DocumentInfo, OnshapeClient, WorkspaceType
from onshape.onshape.schema.assembly import (
    Assembly,
    AssemblyMetadata,
    Part,
    RootAssembly,
    SubAssembly,
)
from onshape.onshape.schema.document import Document
from onshape.onshape.schema.elements import Elements
from onshape.onshape.schema.features import Features
from onshape.onshape.schema.part import PartDynamics, PartMetadata, ThumbnailInfo


class CachedOnshapeApi(OnshapeApi):
    def __init__(
        self,
        client: OnshapeClient,
        cacher: Cacher,
        max_concurrent_requests: int = 1,
        post_wait: float = 0.0,
    ) -> None:
        super().__init__(client, max_concurrent_requests, post_wait)

        self.cacher = cacher

        # Async locks.
        self.master_lock = asyncio.Lock()
        self.stl_locks: dict[str, asyncio.Lock] = {}

    async def get_document(self, did: str) -> Document:
        return await self.cacher(
            f"document_{did}",
            lambda: super(CachedOnshapeApi, self).get_document(did),
            lambda document_str: Document.model_validate_json(document_str),
            lambda document: document.model_dump_json(indent=2),
        )

    async def list_elements(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> Elements:
        return await self.cacher(
            f"elements_{document_id}_{workspace_id}",
            lambda: super(CachedOnshapeApi, self).list_elements(document_id, workspace_id, workspace_type),
            lambda elements_str: Elements.model_validate_json(elements_str),
            lambda elements: elements.model_dump_json(indent=2),
        )

    async def get_first_assembly_id(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> str:
        return await self.cacher(
            f"first_assembly_id_{document_id}_{workspace_id}",
            lambda: super(CachedOnshapeApi, self).get_first_assembly_id(document_id, workspace_id, workspace_type),
            lambda assembly_id: assembly_id,
            lambda assembly_id: assembly_id,
        )

    async def get_assembly(self, document: DocumentInfo, configuration: str = "default") -> Assembly:
        return await self.cacher(
            f"assembly_{document.document_id}_{document.item_id}_{document.element_id}_{configuration}",
            lambda: super(CachedOnshapeApi, self).get_assembly(document, configuration),
            lambda assembly_str: Assembly.model_validate_json(assembly_str),
            lambda assembly: assembly.model_dump_json(indent=2),
        )

    async def get_features(self, asm: RootAssembly | SubAssembly) -> Features:
        return await self.cacher(
            f"features_{asm.key.unique_id}",
            lambda: super(CachedOnshapeApi, self).get_features(asm),
            lambda features_str: Features.model_validate_json(features_str),
            lambda features: features.model_dump_json(indent=2),
        )

    async def get_assembly_metadata(
        self,
        assembly: RootAssembly | SubAssembly,
        configuration: str = "default",
    ) -> AssemblyMetadata:
        return await self.cacher(
            f"assembly_{assembly.key.unique_id}_metadata",
            lambda: super(CachedOnshapeApi, self).get_assembly_metadata(assembly, configuration),
            lambda metadata_str: AssemblyMetadata.model_validate_json(metadata_str),
            lambda metadata: metadata.model_dump_json(indent=2),
        )

    async def get_part_metadata(self, part: Part) -> PartMetadata:
        return await self.cacher(
            f"part_{part.key.unique_id}_metadata",
            lambda: super(CachedOnshapeApi, self).get_part_metadata(part),
            lambda metadata_str: PartMetadata.model_validate_json(metadata_str),
            lambda metadata: metadata.model_dump_json(indent=2),
        )

    async def get_part_mass_properties(self, part: Part) -> PartDynamics:
        return await self.cacher(
            f"part_{part.key.unique_id}_mass_properties",
            lambda: super(CachedOnshapeApi, self).get_part_mass_properties(part),
            lambda props_str: PartDynamics.model_validate_json(props_str),
            lambda props: props.model_dump_json(indent=2),
        )

    async def download_stl(
        self,
        part: Part,
        fp: BinaryIO,
        *,
        units: str = "meter",
        min_facet_width: float | None = None,
        max_facet_width: float | None = None,
        retries: int = 3,
    ) -> None:
        # Each part needs its own lock. We use a master lock to avoid double
        # inserting the same lock.
        async with self.master_lock:
            if part.key.unique_id not in self.stl_locks:
                self.stl_locks[part.key.unique_id] = asyncio.Lock()

        # We need to lock the STL download for the part to avoid downloading
        # the same STL file multiple times.
        async with self.stl_locks[part.key.unique_id]:
            cache_path = self.cacher.cache_dir / f"stl_{part.key.unique_id}.stl"
            if self.cacher._run_fn(cache_path):
                with open(cache_path, "wb") as f:
                    await super(CachedOnshapeApi, self).download_stl(
                        part,
                        f,
                        units=units,
                        min_facet_width=min_facet_width,
                        max_facet_width=max_facet_width,
                        retries=retries,
                    )

        with open(cache_path, "rb") as f:
            fp.write(f.read())

    async def list_thumbnails(self, document: DocumentInfo) -> ThumbnailInfo:
        return await self.cacher(
            f"thumbnails_{document.document_id}_{document.item_id}",
            lambda: super(CachedOnshapeApi, self).list_thumbnails(document),
            lambda thumbnails_str: ThumbnailInfo.model_validate_json(thumbnails_str),
            lambda thumbnails: thumbnails.model_dump_json(indent=2),
        )

    async def download_thumbnail(
        self,
        fp: BinaryIO,
        document: DocumentInfo,
        *,
        width: int = 300,
        height: int = 300,
    ) -> None:
        key = hashlib.md5(f"{document.document_id}_{document.item_id}_{width}_{height}".encode("utf-8")).hexdigest()

        # Each part needs its own lock. We use a master lock to avoid double
        # inserting the same lock.
        async with self.master_lock:
            if key not in self.stl_locks:
                self.stl_locks[key] = asyncio.Lock()

        # We need to lock the STL download for the part to avoid downloading
        # the same STL file multiple times.
        async with self.stl_locks[key]:
            cache_path = self.cacher.cache_dir / f"png_{key}.png"
            if self.cacher._run_fn(cache_path):
                with open(cache_path, "wb") as f:
                    await super(CachedOnshapeApi, self).download_thumbnail(
                        f,
                        document,
                        width=width,
                        height=height,
                    )

        with open(cache_path, "rb") as f:
            fp.write(f.read())
