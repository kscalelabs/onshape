"""OnShape API and client."""

import logging
from pathlib import Path
from typing import Literal

from kol.onshape.client import OnshapeClient
from kol.onshape.schema.assembly import Assembly, AssemblyMetadata, Part, RootAssembly, SubAssembly
from kol.onshape.schema.document import Document
from kol.onshape.schema.elements import Elements, ElementType
from kol.onshape.schema.features import Features
from kol.onshape.schema.part import PartDynamics, PartMetadata

logger = logging.getLogger(__name__)


def escape_url(s: str) -> str:
    return s.replace("/", "%2f").replace("+", "%2b")


DEFAULT_BASE_URL = "https://cad.onshape.com"

WorkspaceType = Literal["w", "v"]


class OnshapeApi:
    def __init__(self, client: OnshapeClient) -> None:
        super().__init__()

        self.client = client

    def get_document(self, did: str) -> Document:
        data = self.client.request("get", f"/api/documents/{did}").json()
        return Document.model_validate(data)

    def list_elements(
        self,
        document_id: str,
        workspace_id: str,
        workspace_type: WorkspaceType = "w",
    ) -> Elements:
        data = self.client.request("get", f"/api/documents/d/{document_id}/{workspace_type}/{workspace_id}/elements")
        return Elements.model_validate(data.json())

    def get_first_assembly_id(self, document_id: str, workspace_id: str, workspace_type: WorkspaceType = "w") -> str:
        elements = self.list_elements(document_id, workspace_id, workspace_type)
        for element in elements.root:
            if element.elementType == ElementType.ASSEMBLY:
                logger.info("Found assembly %s", element.name)
                return element.id
        raise ValueError("Assembly not found")

    def get_assembly(
        self,
        document_id: str,
        workspace_id: str,
        element_id: str,
        workspace_type: WorkspaceType = "w",
        configuration: str = "default",
    ) -> Assembly:
        path = f"/api/assemblies/d/{document_id}/{workspace_type}/{workspace_id}/e/{element_id}"
        data = self.client.request(
            "get",
            path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
        ).json()
        return Assembly.model_validate(data)

    def get_features(self, asm: RootAssembly | SubAssembly) -> Features:
        path = f"/api/assemblies/d/{asm.documentId}/m/{asm.documentMicroversion}/e/{asm.elementId}/features"
        data = self.client.request("get", path).json()
        return Features.model_validate(data)

    def get_assembly_metadata(
        self,
        assembly: RootAssembly | SubAssembly,
        configuration: str = "default",
    ) -> AssemblyMetadata:
        path = f"/api/metadata/d/{assembly.documentId}/m/{assembly.documentMicroversion}/e/{assembly.elementId}"
        data = self.client.request("get", path, query={"configuration": configuration}).json()
        return AssemblyMetadata.model_validate(data)

    def get_part_metadata(self, part: Part) -> PartMetadata:
        path = (
            f"/api/metadata/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/p/{escape_url(part.partId)}"
        )
        data = self.client.request("get", path, query={"configuration": part.configuration}).json()
        return PartMetadata.model_validate(data)

    def get_part_mass_properties(self, part: Part) -> PartDynamics:
        data = self.client.request(
            "get",
            (
                f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
                f"/e/{part.elementId}/partid/{escape_url(part.partId)}/massproperties"
            ),
            query={"configuration": part.configuration, "useMassPropertyOverrides": True},
        ).json()
        return PartDynamics.model_validate(data)

    def download_stl(self, part: Part, output_path: Path) -> None:
        path = (
            f"/api/parts/d/{part.documentId}/m/{part.documentMicroversion}"
            f"/e/{part.elementId}/partid/{escape_url(part.partId)}/stl"
        )
        response = self.client.request(
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
        with open(output_path, "wb") as f:
            f.write(response.content)
