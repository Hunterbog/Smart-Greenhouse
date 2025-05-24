def normalize_interval(value, default_diff=1):
    if value is None:
        return None
    value = str(value).strip()
    import re
    cleaned = re.sub(r'[-\s]+', ' ', value).strip()
    parts = cleaned.split(' ')

    try:
        if len(parts) == 1:
            start = int(round(float(parts[0])))
            end = start + default_diff
        elif len(parts) == 2:
            start = int(round(float(parts[0])))
            end = int(round(float(parts[1])))
        else:
            return None
    except ValueError:
        return None

    if start > end:
        start, end = end, start
    if start == end:
        end = end + 1
    return f"{start}-{end}"
