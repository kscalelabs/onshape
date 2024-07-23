"""OnShape client."""

import base64
import datetime
import hashlib
import hmac
import json
import logging
import os
import random
import re
import string
import urllib.parse
from dataclasses import dataclass
from typing import Any, Literal, cast

import requests

logger = logging.getLogger(__name__)


def escape_url(s: str) -> str:
    return s.replace("/", "%2f").replace("+", "%2b")


ONSHAPE_API_KEY_URL = "https://dev-portal.onshape.com/keys"

DEFAULT_BASE_URL = "https://cad.onshape.com"

Method = Literal["get", "post", "put", "delete"]


WorkspaceType = Literal["w", "v"]


@dataclass
class DocumentInfo:
    document_id: str
    item_kind: WorkspaceType
    item_id: str
    element_id: str


class OnshapeClient:
    def __init__(
        self,
        access_key: str | None = None,
        secret_key: str | None = None,
        base_url: str = DEFAULT_BASE_URL,
    ) -> None:
        super().__init__()

        if access_key is None:
            if "ONSHAPE_ACCESS_KEY" not in os.environ:
                raise ValueError(
                    "The ONSHAPE_ACCESS_KEY environment variable is not set. You must first generate an API key "
                    f"by navigating to the following URL: {ONSHAPE_API_KEY_URL}, then set the ONSHAPE_ACCESS_KEY "
                    "and ONSHAPE_SECRET_KEY environment variables."
                )
            access_key = os.environ["ONSHAPE_ACCESS_KEY"]

        if secret_key is None:
            if "ONSHAPE_SECRET_KEY" not in os.environ:
                raise ValueError(
                    "The ONSHAPE_SECRET_KEY environment variable is not set. You must first generate an API key "
                    f"by navigating to the following URL: {ONSHAPE_API_KEY_URL}, then set the ONSHAPE_ACCESS_KEY "
                    "and ONSHAPE_SECRET_KEY environment variables."
                )
            secret_key = os.environ["ONSHAPE_SECRET_KEY"]

        self.access_key = access_key.encode("utf-8")
        self.secret_key = secret_key.encode("utf-8")
        self.base_url = base_url

    def parse_url(self, document_url: str) -> DocumentInfo:
        url_match = re.match(rf"{self.base_url}/documents/([\w\d]+)/(w|v)/([\w\d]+)/e/([\w\d]+)", document_url)
        if url_match is None:
            raise ValueError(f"Invalid document URL: {document_url}")
        document_id = url_match.group(1)
        item_kind = cast(WorkspaceType, url_match.group(2))
        item_id = url_match.group(3)
        element_id = url_match.group(4)
        return DocumentInfo(document_id, item_kind, item_id, element_id)

    def _make_nonce(self) -> str:
        """Generate a unique ID for the request, 25 chars in length.

        Returns:
            The unique nonce to use.
        """
        chars = string.digits + string.ascii_letters
        nonce = "".join(random.choice(chars) for i in range(25))
        logger.debug("Created nonce: %s", nonce)
        return nonce

    def _make_auth(
        self,
        method: str,
        date: str,
        nonce: str,
        path: str,
        query: dict[str, Any] = {},
        ctype: str = "application/json",
    ) -> str:
        """Create the request signature to authenticate.

        Args:
            method: The HTTP method.
            date: The date header string.
            nonce: The cryptographic nonce.
            path: The URL pathname.
            query: The URL query string in key-value pairs.
            ctype: The HTTP Content-Type.

        Returns:
            The request signature.
        """
        query_str = urllib.parse.urlencode(query)
        hmac_str = f"{method}\n{nonce}\n{date}\n{ctype}\n{path}\n{query_str}\n".lower().encode("utf-8")
        logger.debug("HMAC string: %s", hmac_str)
        signature = base64.b64encode(hmac.new(self.secret_key, hmac_str, digestmod=hashlib.sha256).digest())
        auth = "On " + self.access_key.decode("utf-8") + ":HmacSHA256:" + signature.decode("utf-8")
        logger.debug("Auth header: %s", auth)
        return auth

    def _make_headers(
        self,
        method: str,
        path: str,
        query: dict[str, Any] = {},
        headers: dict[str, str] = {},
    ) -> dict[str, str]:
        """Creates a headers object to sign the request.

        Args:
            method: The HTTP method.
            path: The URL pathname.
            query: The URL query string in key-value pairs.
            headers: Additional headers to include.

        Returns:
            Dictionary containing all headers
        """
        date = datetime.datetime.now(datetime.UTC).strftime("%a, %d %b %Y %H:%M:%S GMT")
        nonce = self._make_nonce()
        ctype = headers.get("Content-Type", "application/json")
        auth = self._make_auth(method, date, nonce, path, query=query, ctype=ctype)
        req_headers = {
            "Content-Type": "application/json",
            "Date": date,
            "On-Nonce": nonce,
            "Authorization": auth,
            "User-Agent": "Onshape Python Sample App",
            "Accept": "application/json",
        }
        # User-provided headers override defaults.
        for h in headers:
            req_headers[h] = headers[h]
        return req_headers

    def request(
        self,
        method: Method,
        path: str,
        query: dict[str, Any] = {},
        headers: dict[str, str] = {},
        body: dict[str, str] | str = {},
        base_url: str | None = None,
    ) -> requests.Response:
        """Issues a request to Onshape.

        Args:
            method: The HTTP method.
            path: The URL pathname.
            query: The URL query string in key-value pairs.
            headers: Additional headers to include.
            body: The request body.
            base_url: The base URL to use.

        Returns:
            The response from Onshape.
        """
        req_headers = self._make_headers(method, path, query, headers)
        if base_url is None:
            base_url = self.base_url
        url = base_url + path + "?" + urllib.parse.urlencode(query)

        body = json.dumps(body) if isinstance(body, dict) else body
        response = requests.request(method, url, headers=req_headers, data=body, allow_redirects=False, stream=True)

        if response.status_code == 307:
            location = urllib.parse.urlparse(response.headers["Location"])
            querystring = urllib.parse.parse_qs(location.query)
            logger.debug("Request redirected to: %s", location.geturl())
            new_query = {}
            new_base_url = location.scheme + "://" + location.netloc
            for key in querystring:
                new_query[key] = querystring[key][0]  # won't work for repeated query params
            return self.request(method, location.path, query=new_query, headers=headers, base_url=new_base_url)

        elif not 200 <= response.status_code <= 206:
            logger.error("Got response %d for %s, details: %s", response.status_code, path, response.text)

            if response.status_code == 403:
                logger.warning("Check that your authentication information is correct")

        else:
            logger.debug("Got response %d for %s", response.status_code, path)

        return response
