# -*- coding: utf-8 -*-

DEFAULT_URBAN_CANDIDATE_GROUPS = [
    "1차선",
    "2차선(왼쪽)",
    "2차선(오른쪽)",
    "3차선(중간)",
    "3차선(왼쪽)",
    "3차선(오른쪽)",
    "일방통행",
]


def empty_candidate_link_groups():
    return {name: [] for name in DEFAULT_URBAN_CANDIDATE_GROUPS}


def ensure_candidate_link_groups(zone_data):
    groups = zone_data.get("candidate_link_groups")
    if not isinstance(groups, dict):
        groups = empty_candidate_link_groups()
        zone_data["candidate_link_groups"] = groups

    for name in DEFAULT_URBAN_CANDIDATE_GROUPS:
        groups.setdefault(name, [])

    for name, links in list(groups.items()):
        if links is None:
            groups[name] = []
        elif not isinstance(links, list):
            groups[name] = list(links)

    return groups


def iter_unique_links(links):
    seen = set()
    for link_id in links:
        if not link_id or link_id in seen:
            continue
        seen.add(link_id)
        yield link_id


def flatten_candidate_links(zone_data, include_legacy_route_links=True):
    links = []

    if include_legacy_route_links:
        links.extend(zone_data.get("route_links", []) or [])

    groups = ensure_candidate_link_groups(zone_data)
    for group_links in groups.values():
        links.extend(group_links or [])

    return list(iter_unique_links(links))


def build_link_group_lookup(zone_data):
    groups = ensure_candidate_link_groups(zone_data)
    lookup = {}
    for group_name, links in groups.items():
        for link_id in links or []:
            lookup[link_id] = group_name
    return lookup


def find_link_group(zone_data, link_id):
    groups = ensure_candidate_link_groups(zone_data)
    for group_name, links in groups.items():
        if link_id in (links or []):
            return group_name
    return None


def add_link_to_group(zone_data, group_name, link_id):
    groups = ensure_candidate_link_groups(zone_data)
    if group_name not in groups:
        raise KeyError(f"Unknown candidate link group: {group_name}")

    existing_group = find_link_group(zone_data, link_id)
    if existing_group:
        return False, existing_group

    groups[group_name].append(link_id)
    return True, group_name
