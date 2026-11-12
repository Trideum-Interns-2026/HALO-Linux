"""The one place the bridge talks HTTP. Tests replace it with a fake."""
import urllib.error
import urllib.request

from ui.telemetry_mapper import encode_payload


class HttpPoster:
    def __init__(self, url: str, timeout_s: float):
        self._url = url
        self._timeout_s = timeout_s

    def post(self, payload: dict) -> None:
        """POST the payload as JSON. Raises OSError (incl. URLError) on any failure."""
        request = urllib.request.Request(
            self._url, data=encode_payload(payload),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self._timeout_s) as response:
                response.read()
        except urllib.error.HTTPError as error:  # HTTPError is an OSError, keep the status
            raise OSError(f"HTTP {error.code} from {self._url}") from error
