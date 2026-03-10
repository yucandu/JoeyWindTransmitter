import json
import re

with open('grafana.json', 'r') as f:
    data = json.load(f)

metrics = []

def extract_metrics(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "rawSql":
                sql = v
                # Extract name (before [)
                m = re.search(r'AS\s+"([^"\[]+)', sql, re.IGNORECASE)
                name = m.group(1).strip() if m else "Unknown"
                # Extract WHERE clause
                where = re.search(r'WHERE\s*\((.*?)\)', sql, re.IGNORECASE)
                where_clause = where.group(1) if where else ""
                # Compose query
                query = f"SELECT value FROM reporting_average_minute WHERE {where_clause} ORDER BY time DESC LIMIT 1"
                metrics.append((name, query))
            else:
                extract_metrics(v)
    elif isinstance(obj, list):
        for item in obj:
            extract_metrics(item)

extract_metrics(data)

for name, query in metrics:
    print(f'    {{"{name}", "{query}", 0}},')