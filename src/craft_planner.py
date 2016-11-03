# P5
#
# CHARLOTTE KIRBY - 1382261
# URIAN LEE - 1454351
#

import json
from collections import namedtuple, defaultdict, OrderedDict
from timeit import default_timer as time
from heapq import heappush, heappop
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
            for r, q in requireables:
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
            for c, b in consumables:
                # c (consumed), b (boolean)
                if c not in state_copy.keys():
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
            for p, q in productions:
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
            for c, q in consumption:
                # c (consumed), q (quantity)
                nq = next_state[c]
                nq -= q
                next_state.update({p: nq})

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
        for g, q in goal:
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


def heuristic(state):
    # Implement your heuristic here!
    return 0


def search(graph, state, is_goal, limit, heuristic):
    print('TESTING')
    start_time = time()

    # frontier = PriorityQueue()
    queue = []  # HEAPQUEUE

    # ----------------- 1 -------------------------------------
    # MAY BE A PROBLEM HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    initial_name = ''
    initial_state = state
    initial_cost = 0
    initial_baby = (initial_name, initial_state, initial_cost)

    # ------------------------ 2 ---------------------------------
    heappush(queue, initial_baby)  # MIGHT NEED TO SWITCH tuples!!!!

    came_from = {}
    cost_so_far = {}
    came_from[initial_baby] = None
    cost_so_far[initial_baby] = 0

    # Implement your search here! Use your heuristic here!
    # When you find a path to the goal return a list of tuples [(state, action)]
    # representing the path. Each element (tuple) of the list represents a state
    # in the path and the action that took you to this state

    # while not frontier.empty():
    while time() - start_time < limit and len(queue) is not 0:
        rule_name, res_state, rule_cost = heappop(queue)
        popped_baby = (rule_name, res_state, rule_cost)
        if is_goal:
            backtracker =

        graph_list = graph()  # (rule's name, resulting state, cost of rule)
        for g in graph_list:
            g_name = g[0]
            g_state = g[1]
            g_cost = g[2]
            # --------------------- 3 -------------------------------
            new_cost = cost_so_far[rule_name] + g_cost  # from current to next
            if g_name not in cost_so_far or new_cost < cost_so_far[g_name]:
                cost_so_far[g_name] = new_cost
                priority = new_cost + heuristic(state)  # (goal, next)
                new_baby = (g_name, g_state, priority)
                came_from[g_name] = current
                heappush(queue, new_baby)
                came_from[g_name] = popped_baby
        #           new_cost = cost_so_far[current] + graph.cost(current, next)
        #           if next not in cost_so_far or new_cost < cost_so_far[next]:
        #                cost_so_far[next] = new_cost
        #                priority = new_cost + heuristic(goal, next)
        #                frontier.put(next, priority)
        #                came_from[next] = current
        pass

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
    resulting_plan = search(graph, state, is_goal, 30, heuristic)

    if resulting_plan:
        # Print resulting plan
        for state, action in resulting_plan:
            print('\t', state)
            print(action)
