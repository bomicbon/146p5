# P5
#
# CHARLOTTE KIRBY - 1382261
# URIAN LEE - 1454351
#

import json
from collections import namedtuple, defaultdict, OrderedDict
from timeit import default_timer as time
from heapq import heappush, heappop
from math import inf
Recipe = namedtuple('Recipe', ['name', 'check', 'effect', 'cost'])


class State(OrderedDict):
    """ This class is a thin wrapper around an OrderedDict, which is simply a dictionary which keeps the order in
        which elements are added (for consistent key-value pair comparisons). Here, we have provided functionality
        for hashing, should you need to use a state as a key in another dictionary, e.g. distance[state] = 5. By
        default, dictionaries are not hashable. Additionally, when the state is converted to a string, it removes
        all items with quantity 0.

        Use of this state representation is optional, should you prefer another.
    """

    def __key(self):
        return tuple(self.items())
    # (('apple', 3), ('berry', 2))

    def __hash__(self):
        return hash(self.__key())

    def __lt__(self, other):
        return self.__key() < other.__key()
    # len(self) < len(other)

    def copy(self):
        new_state = State()
        new_state.update(self)
        return new_state

    def __str__(self):
        return str(dict(item for item in self.items() if item[1] > 0))

    def vk(self):
        res = dict((v, k) for k, v in self.items())
        return res


def make_checker(rule):
    # Implement a function that returns a function to determine whether a state meets a
    # rule's requirements. This code runs once, when the rules are constructed before
    # the search is attempted.
    def check(state):
        # This code is called by graph(state) and runs millions of times.
        # Tip: Do something with rule['Consumes'] and rule['Requires'].
        state_copy = state.copy()

        # Return False if state doesn't have what's REQUIRED
        if 'Requires' in rule.keys():
            requireables = rule['Requires']
            for r in requireables:
                q = requireables[r]
                # r (required), q (quantity)
                if r not in state_copy.keys():
                    return False
                elif r in state_copy.keys():
                    quantity = state_copy[r]
                    if quantity < q:
                        return False
                    else:
                        pass

        # Return False if state doesn't have what's CONSUMED
        if 'Consumes' in rule.keys():
            consumables = rule['Consumes']
            for c in consumables:
                q = consumables[c]
                # c (consumed), q (quantity)
                if c not in state_copy.keys():
                    return False
                else:
                    quantity = state_copy[c]
                    if quantity < q:
                        return False
                    else:
                        pass

        # PASSED THE SCREENING - state has the stuff
        return True
    return check


def make_effector(rule):
    # Implement a function that returns a function which transitions from state to
    # new_state given the rule. This code runs once, when the rules are constructed
    # before the search is attempted.

    def effect(state):
        # This code is called by graph(state) and runs millions of times
        # Tip: Do something with rule['Produces'] and rule['Consumes'].

        next_state = state.copy()

        # add PRODUCES to state
        if 'Produces' in rule.keys():
            productions = rule['Produces']
            for p in productions:
                q = productions[p]
                # p (product), q (quantity)
                if p not in next_state:  # Add Product
                    next_state.update({p: q})
                else:  # Update Quantity
                    nq = next_state[p]
                    nq += q
                    next_state.update({p: nq})

        # remove CONSUMES from state
        if 'Consumes' in rule.keys():
            consumption = rule['Consumes']
            for c in consumption:
                q = consumption[c]
                # c (consumed), q (quantity)
                next_state[c] -= q

        return next_state

    return effect


def make_goal_checker(goal):
    # Implement a function that returns a function which checks if the state has
    # met the goal criteria. This code runs once, before the search is
    # attempted.

    def is_goal(state):
        # This code is used in the search process and may be called millions of
        # times.
        state_copy = state.copy()
        for g in goal:
            q = goal[g]
            # Goal item not obtained
            if g not in state_copy.keys():
                return False
            else:
                # Goal quantity not reached
                if state_copy[g] <= q:
                    return False

        # Screening PASSED. Goal is reached!
        return True

    return is_goal


def graph(state):
    # Iterates through all recipes/rules, checking which are valid in the given state.
    # If a rule is valid, it returns the rule's name, the resulting state after application
    # to the given state, and the cost for the rule.
    for r in all_recipes:
        if r.check(state):
            yield (r.name, r.effect(state), r.cost)


def heuristic(state, rule_name):
    # if we have a tool, no need to search to make another one
    # crafting a tool will put a space after the term; using it will end the name with that term
    if "axe " in rule_name and "axe" in state.keys():
        return float(inf)
    #inefficient to break this up, but more readable to me...
    elif "furnace " in rule_name and "furnace" in state.keys():
        return float(inf)
    elif "bench " in rule_name and "bench" in state.keys():
        return float(inf)
    # don't bother with any iron axe path, there's no benefit over stone axe
    elif "iron_axe" in state.keys():
        return float(inf)
    return 0


def search(graph, state, is_goal, limit, heuristic):
    start_time = time()
    queue = []  # HEAPQUEUE
    initial_name = 'initial'
    initial_state = state
    initial_hash = state.__hash__()
    initial_cost = 0
    initial_baby = (initial_name, initial_state, initial_cost)
    heappush(queue, initial_baby)
    came_from = {}
    cost_so_far = {}
    action_hash = {}
    came_from[initial_hash] = None
    cost_so_far[initial_name] = 0

    # Implement your search here! Use your heuristic here!
    # When you find a path to the goal return a list of tuples [(state, action)]
    # representing the path. Each element (tuple) of the list represents a state
    # in the path and the action that took you to this state

    while time() - start_time < limit and len(queue) is not 0:
        rule_name, res_state, rule_cost = heappop(queue)
        res_state_hash = res_state.__hash__()
        print('')
        print('NAME: ', rule_name)
        print('STATE: ', res_state)
        print('RULE-COST: ', rule_cost)
        if is_goal is True:
            action_path = []
            hashKey = state.__hash__()
            while hashKey is not initial_hash:
                action_path.append(action_hash[hashKey])
                hashKey = came_from[hashKey]
            return action_path

        # (rule's name, resulting state, cost of rule)
        graph_list = graph(res_state)
        for g in graph_list:
            g_name = g[0]
            g_state = g[1]
            g_hash = g_state.__hash__()
            g_cost = g[2]
            print('(n,s,c): ', '(',  g_name, ',', g_state, ',', g_cost, ')')
            new_cost = cost_so_far[rule_name] + g_cost  # from current to next
            print('new_cost: ', new_cost)
            # if g_name not in cost_so_far or new_cost < cost_so_far[g_name]:
            cost_so_far[g_name] = new_cost
            priority = new_cost + heuristic(state, g_name)  # (goal, next)
            came_from[g_hash] = res_state_hash
            action_hash[g_hash] = (g_state, g_name)
            new_baby = (g_name, g_state, priority)
            heappush(queue, new_baby)

    # Failed to find a path
    print(time() - start_time, 'seconds.')
    print("Failed to find a path from", state, 'within time limit.')
    return None

if __name__ == '__main__':
    with open('Crafting.json') as f:
        Crafting = json.load(f)

    # List of items that can be in your inventory:
    print('All items:', Crafting['Items'])

    # List of items in your initial inventory with amounts:
    print('Initial inventory:', Crafting['Initial'])

    # List of items needed to be in your inventory at the end of the plan:
    print('Goal:', Crafting['Goal'])

    # Dict of crafting recipes (each is a dict):
    print('Example recipe:', 'craft stone_pickaxe at bench ->',
          Crafting['Recipes']['craft stone_pickaxe at bench'])

    # Build rules
    all_recipes = []

    for name, rule in Crafting['Recipes'].items():
        print(name)
        print(rule)
        checker = make_checker(rule)
        effector = make_effector(rule)
        recipe = Recipe(name, checker, effector, rule['Time'])
        all_recipes.append(recipe)

    # Create a function which checks for the goal
    is_goal = make_goal_checker(Crafting['Goal'])

    # Initialize first state from initial inventory
    state = State({key: 0 for key in Crafting['Items']})
    state.update(Crafting['Initial'])  # Dictionary to be added to state

    # Search for a solution
    resulting_plan = search(graph, state, is_goal, 0.1, heuristic)

    if resulting_plan:
        # Print resulting plan
        for state, action in resulting_plan:
            print('\t', state)
            print(action)
