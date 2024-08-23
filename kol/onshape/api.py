"""OnShape API and client."""

import logging
from typing import BinaryIO

from kol.onshape.client import DocumentInfo, OnshapeClient, WorkspaceType
from kol.onshape.schema.assembly import (
    Assembly,
    AssemblyMetadata,
    Part,
    RootAssembly,
    SubAssembly,
)
from kol.onshape.schema.document import Document
from kol.onshape.schema.elements import Elements, ElementType
from kol.onshape.schema.features import Features
from kol.onshape.schema.part import PartDynamics, PartMetadata

logger = logging.getLogger(__name__)


def escape_url(s: str) -> str:
    return s.replace("/", "%2f").replace("+", "%2b")


class OnshapeApi:
    def __init__(self, client: OnshapeClient) -> None:
        super().__init__()

        self.client = client

    def parse_url(self, document_url: str) -> DocumentInfo:
        return self.client.parse_url(document_url)

    async def get_document(self, did: str) -> Document:
        response = await self.client.request("get", f"/api/documents/{did}")
        data = response.json()
        return Document.model_validate(data)

    async def list_elements(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> Elements:
        response = await self.client.request(
            "get",
            f"/api/documents/d/{document_id}/{workspace_type}/{workspace_id}/elements",
        )
        return Elements.model_validate(response.json())

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
            f"/api/assemblies/d/{document.document_id}/"
            f"{document.item_kind}/{document.item_id}/e/{document.element_id}"
        )
        response = await self.client.request(
            "get",
            path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
        )
        return Assembly.model_validate(response.json())

    async def get_features(self, asm: RootAssembly | SubAssembly) -> Features:
        path = f"/api/assemblies/d/{asm.documentId}/m/{asm.documentMicroversion}/e/{asm.elementId}/features"
        response = await self.client.request("get", path)
        return Features.model_validate(response.json())

    async def get_assembly_metadata(
        self,
        assembly: RootAssembly | SubAssembly,
        configuration: str = "default",
    ) -> AssemblyMetadata:
        path = f"/api/metadata/d/{assembly.documentId}/m/{assembly.documentMicroversion}/e/{assembly.elementId}"
        response = await self.client.request("get", path, query={"configuration": configuration})
        return AssemblyMetadata.model_validate(response.json())

    async def get_part_metadata(self, part: Part) -> PartMetadata:
        path = (
            f"/api/metadata/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/p/{escape_url(part.partId)}"
        )
        response = await self.client.request("get", path, query={"configuration": part.configuration})
        return PartMetadata.model_validate(response.json())

    async def get_part_mass_properties(self, part: Part) -> PartDynamics:
        response = await self.client.request(
            "get",
            (
                f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
                f"/e/{part.elementId}/partid/{escape_url(part.partId)}/massproperties"
            ),
            query={"configuration": part.configuration, "useMassPropertyOverrides": True},
        )
        return PartDynamics.model_validate(response.json())

    async def download_stl(self, part: Part, fp: BinaryIO) -> None:
        path = (
            f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/partid/{escape_url(part.partId)}/stl"
        )
        response = await self.client.request(
            "get",
            path,
            query={
                "mode": "binary",
                "grouping": True,
                "units": "meter",
                "configuration": part.configuration,
            },
            headers={"Accept": "*/*"},
        )
        response.raise_for_status()
        fp.write(response.content)
