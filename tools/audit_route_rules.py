"""Offline baseline for a rule refactor; never connects to or writes to a robot."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modeling_task_generator import generate_task_plan
from route_point_policy import EXECUTION_POINT_MERGE_CM


def fingerprint(value):
    raw = json.dumps(value, sort_keys=True, ensure_ascii=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode("ascii")).hexdigest()


def audit(root):
    models = {}
    for path in sorted(root.rglob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8-sig"))
        except (ValueError, OSError):
            continue
        if not isinstance(data, dict) or not data.get("groups"):
            continue
        if (data.get("taskPreview") or {}).get("status") != "ready":
            continue
        models.setdefault(fingerprint(data), (path, data))
    cases = []
    for digest, (path, model) in models.items():
        for return_home in (True, False):
            case = {"modelHash": digest, "file": str(path),
                    "name": model.get("name") or model.get("id"),
                    "returnToOrigin": return_home}
            candidate = copy.deepcopy(model)
            try:
                plan = generate_task_plan(candidate, now=0, return_to_origin=return_home)
                tasks = plan["tasks"]
                case.update({"resultHash": fingerprint(plan), "summary": plan["summary"],
                             "shortTasks": [{k: t.get(k) for k in (
                                 "id", "length", "mode", "startX", "startY", "endX", "endY",
                                 "source", "turnAtStart", "stopAtEnd", "preserveRecordedPath")}
                                 for t in tasks if t["length"] <= EXECUTION_POINT_MERGE_CM]})
            except Exception as error:
                case["error"] = type(error).__name__ + ": " + str(error)
            case["inputUnchanged"] = fingerprint(candidate) == digest
            cases.append(case)
    return {"modelCount": len(models), "cases": cases}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("backup_root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--compare", type=Path)
    args = parser.parse_args()
    report = audit(args.backup_root)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    cases = report["cases"]
    print(json.dumps({"models": report["modelCount"], "variants": len(cases),
                      "generated": sum("resultHash" in c for c in cases),
                      "errors": sum("error" in c for c in cases),
                      "variantsWithShortTasks": sum(bool(c.get("shortTasks")) for c in cases),
                      "inputsUnchanged": all(c["inputUnchanged"] for c in cases)}, ensure_ascii=True))
    if args.compare:
        before = json.loads(args.compare.read_text(encoding="utf-8"))
        unchanged = before == report
        print("BASELINE_IDENTICAL=" + str(unchanged))
        return 0 if unchanged else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
