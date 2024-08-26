"""Defines a cached version of the Onshape API."""

from kol.onshape.api import OnshapeApi
from kol.onshape.cacher import Cacher
from kol.onshape.client import DocumentInfo, OnshapeClient, WorkspaceType
from kol.onshape.schema.assembly import (
    Assembly,
    AssemblyMetadata,
    Part,
    RootAssembly,
    SubAssembly,
)
from kol.onshape.schema.document import Document
from kol.onshape.schema.elements import Elements
from kol.onshape.schema.features import Features
from kol.onshape.schema.part import PartDynamics, PartMetadata


class CachedOnshapeApi(OnshapeApi):
    def __init__(self, client: OnshapeClient, cacher: Cacher) -> None:
        super().__init__(client)

        self.cacher = cacher

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
