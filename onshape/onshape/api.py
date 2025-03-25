"""OnShape API and client."""

import asyncio
import logging
from typing import Any, BinaryIO, Literal, Mapping

from httpx import AsyncClient

from onshape.onshape.client import DocumentInfo, OnshapeClient, WorkspaceType
from onshape.onshape.schema.assembly import (
    Assembly,
    AssemblyMetadata,
    Part,
    RootAssembly,
    SubAssembly,
)
from onshape.onshape.schema.document import Document
from onshape.onshape.schema.elements import Elements, ElementType
from onshape.onshape.schema.features import Features
from onshape.onshape.schema.part import PartDynamics, PartMetadata, ThumbnailInfo

logger = logging.getLogger(__name__)

RETRY_STATUS_CODES = (429, 500, 502, 503, 504)


def escape_url(s: str) -> str:
    return s.replace("/", "%2f").replace("+", "%2b")


class OnshapeApi:
    def __init__(
        self,
        client: OnshapeClient,
        max_concurrent_requests: int = 1,
        post_wait: float = 0.0,
    ) -> None:
        super().__init__()
        self.client = client
        self.semaphore = asyncio.Semaphore(max_concurrent_requests)
        self.post_wait = post_wait

    def parse_url(self, document_url: str) -> DocumentInfo:
        return self.client.parse_url(document_url)

    async def _request(
        self,
        method: Literal["get", "post", "put", "delete"],
        path: str,
        query: Mapping[str, Any] | None = None,
        headers: Mapping[str, str] | None = None,
        body: Mapping[str, Any] | None = None,
        base_url: str | None = None,
    ) -> dict:
        async with self.semaphore:
            async with self.client.request(
                method=method,
                path=path,
                query={} if query is None else query,
                headers={} if headers is None else headers,
                body={} if body is None else body,
                base_url=base_url,
            ) as response:
                await response.aread()
                response.raise_for_status()
                response_data = response.json()

        if self.post_wait > 0.0:
            await asyncio.sleep(self.post_wait)

        return response_data

    async def get_document(self, did: str) -> Document:
        data = await self._request("get", f"/api/documents/{did}")
        return Document.model_validate(data)

    async def list_elements(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> Elements:
        data = await self._request(
            "get",
            f"/api/documents/d/{document_id}/{workspace_type}/{workspace_id}/elements",
        )
        return Elements.model_validate(data)

    async def get_first_assembly_id(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> str:
        elements = await self.list_elements(document_id, workspace_id, workspace_type)
        for element in elements.root:
            if element.elementType == ElementType.ASSEMBLY:
                logger.info("Found assembly %s", element.name)
                return element.id
        raise ValueError("Assembly not found")

    async def get_assembly(self, document: DocumentInfo, configuration: str = "default") -> Assembly:
        path = (
            f"/api/assemblies/d/{document.document_id}/{document.item_kind}/{document.item_id}/e/{document.element_id}"
        )
        data = await self._request(
            "get",
            path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
        )
        return Assembly.model_validate(data)

    async def get_features(self, asm: RootAssembly | SubAssembly) -> Features:
        path = f"/api/assemblies/d/{asm.documentId}/m/{asm.documentMicroversion}/e/{asm.elementId}/features"
        data = await self._request("get", path)
        return Features.model_validate(data)

    async def get_assembly_metadata(
        self,
        assembly: RootAssembly | SubAssembly,
        configuration: str = "default",
    ) -> AssemblyMetadata:
        path = f"/api/metadata/d/{assembly.documentId}/m/{assembly.documentMicroversion}/e/{assembly.elementId}"
        data = await self._request("get", path, query={"configuration": configuration})
        return AssemblyMetadata.model_validate(data)

    async def get_part_metadata(self, part: Part) -> PartMetadata:
        path = (
            f"/api/metadata/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/p/{escape_url(part.partId)}"
        )
        data = await self._request("get", path, query={"configuration": part.configuration})
        return PartMetadata.model_validate(data)

    async def get_part_mass_properties(self, part: Part) -> PartDynamics:
        path = (
            f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/partid/{escape_url(part.partId)}/massproperties"
        )
        data = await self._request(
            "get",
            path,
            query={
                "configuration": part.configuration,
                "useMassPropertyOverrides": True,
                "elementMicroversionId": part.documentMicroversion,
            },
        )
        return PartDynamics.model_validate(data)

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
        path = (
            f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/partid/{escape_url(part.partId)}/stl"
        )
        query = {
            "mode": "binary",
            "grouping": True,
            "units": units,
            "configuration": part.configuration,
        }
        if min_facet_width is not None:
            query["minFacetWidth"] = min_facet_width
        if max_facet_width is not None:
            query["maxFacetWidth"] = max_facet_width

        for _ in range(retries):
            async with self.semaphore:
                async with self.client.request(
                    "get",
                    path,
                    query=query,
                    headers={"Accept": "*/*"},
                ) as response:
                    data = await response.aread()
                    if response.status_code in RETRY_STATUS_CODES:
                        logger.warning("Retrying download for %s (status code %d)", part.partId, response.status_code)
                        continue
                    response.raise_for_status()
                    fp.write(data)

            if self.post_wait > 0.0:
                await asyncio.sleep(self.post_wait)
            break
        else:
            raise RuntimeError(
                f"Failed to download STL for part {part.partId} after {retries} retries; "
                "are you connected to the internet, and is the Onshape API accessible?"
            )

    async def list_thumbnails(self, document: DocumentInfo) -> ThumbnailInfo:
        path = f"/api/thumbnails/d/{document.document_id}/{document.item_kind}/{document.item_id}"
        data = await self._request("get", path)
        return ThumbnailInfo.model_validate(data)

    async def download_thumbnail(
        self,
        fp: BinaryIO,
        document: DocumentInfo,
        *,
        width: int = 300,
        height: int = 300,
    ) -> None:
        thumbnail_info = await self.list_thumbnails(document)

        size = f"{width}x{height}"
        for thumbnail in thumbnail_info.sizes:
            if thumbnail.size == size:
                break
        else:
            choices = ", ".join(thumbnail.size for thumbnail in thumbnail_info.sizes)
            raise ValueError(f"Thumbnail size {size} not found! Choices are {choices}")

        async with self.semaphore:
            async with AsyncClient(
                timeout=self.client.timeout,
                follow_redirects=False,
            ) as client:
                async with client.stream(
                    "get",
                    thumbnail.href,
                    headers={"Accept": thumbnail.mediaType},
                    data={},
                ) as response:
                    response.raise_for_status()
                    async for chunk in response.aiter_bytes():
                        fp.write(chunk)

        if self.post_wait > 0.0:
            await asyncio.sleep(self.post_wait)
