# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import argparse
import uvicorn
from schunk_gripper_dummy.dummy import Dummy
from fastapi import FastAPI, Request, Form, Body, BackgroundTasks, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import string


def create_webserver(dummy: Dummy):
    webserver = FastAPI()
    clients: list[str] = []

    webserver.add_middleware(
        CORSMiddleware,
        allow_origins=clients,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @webserver.post("/adi/update.json")
    async def update(
        inst: str = Form(...),
        value: str = Form(...),
        elem: Optional[int] = Form(None),
        callback: Optional[str] = Form(None),
        background_tasks: BackgroundTasks = BackgroundTasks(),
    ):
        msg = {"inst": inst, "value": value, "elem": elem, "callback": callback}
        if msg["inst"] not in dummy.data:
            return {"result": 1}
        if not all(digit in string.hexdigits for digit in str(msg["value"])):
            return {"result": 1}
        background_tasks.add_task(dummy.update, msg)
        return {"result": 0}

    @webserver.post("/adi/events.json")
    async def events(events: dict = Body(...)):
        return dummy.handle_events(events=events)

    @webserver.get("/adi/data.json")
    async def get(request: Request):
        if not dummy.reachable:
            raise HTTPException(status_code=503)
        params = dict(request.query_params)
        return dummy.get_data(params)

    return webserver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--gripper", type=str, default="EGU_60_EI_M_B")
    args = parser.parse_args()

    dummy = None
    try:
        dummy = Dummy(args.gripper)
        dummy.start()
        webserver = create_webserver(dummy)
        uvicorn.run(webserver, host="0.0.0.0", port=args.port)
    except ValueError:
        pass
    except KeyboardInterrupt:
        print("Server stopped by user.")
    finally:
        if dummy:
            dummy.stop()


if __name__ == "__main__":
    main()
