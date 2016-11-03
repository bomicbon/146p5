def heuristic(state, rule_name):
    # if we have a tool, no need to search to make another one
    # crafting a tool will put a space after the term; using it will end the name with that term
    if "axe " in rule_name and "axe" in state.keys():
        return math.inf
    #inefficient to break this up, but more readable to me...
    elif "furnace " in rule_name and "furnace" in state.keys():
        return math.inf
    elif "bench " in rule_name and "bench" in state.keys():
        return math.inf
    # don't bother with any iron axe path, there's no benefit over stone axe
    elif "iron_axe" in state.keys():
        return math.inf
    return 0