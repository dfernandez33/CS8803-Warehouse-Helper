import requests
from typing import Dict, List
from planning.actions import Action, Navigate, PickUp, DropOff, ActionType, Item, Location


class Planner:
    def __init__(self, domain_file_path, problem_template_file):
        self.domain = open(domain_file_path, 'r').read()
        self.problem_template = open(problem_template_file, 'r').read()

    def get_plan(self, query: Dict) -> List[Action]:
        """
        This function provides you a valid plan to execute the provided query
        :param query: This must be a dictionary with keys: {'cups', 'beers', 'balls'} with the values for each ranging
        from 0-3 corresponding to how many of each item you are requesting
        :return: returns a list of actions to be executed in sequence to complete the plan.
        """
        problem = self.__parse_query(query)
        data = {'domain': self.domain,
                'problem': problem}

        resp = requests.post('http://solver.planning.domains/solve',
                             verify=False, json=data).json()

        plan = resp['result']['plan']
        parsed_plan = self.__parse_plan(plan)

        return parsed_plan

    def __parse_query(self, query: Dict):
        goal_propositions = []
        # always add 1 to index since objects in pddl start at 1 not 0
        for i in range(query['cups']):  # build propositions for cups
            goal_propositions.append('(in cup_{} cargo_bay)'.format(i+1))

        for i in range(query['beers']):  # build propositions for beers
            goal_propositions.append('(in beer_{} cargo_bay)'.format(i+1))

        for i in range(query['balls']):  # build propositions for balls
            goal_propositions.append('(in ball_{} cargo_bay)'.format(i+1))

        complete_problem = self.problem_template + '\n'
        complete_problem += '(:goal (and '
        for proposition in goal_propositions:  # add each proposition to the problem's goal
            complete_problem += proposition + ' '
        complete_problem += ')))'  # add closing brackets to pddl file

        return complete_problem

    def __parse_plan(self, plan: Dict):
        parsed_plan = []
        # actions have the form '(command param1 param2)'
        for action in plan:
            human_readable_action = action['name']
            human_readable_action = human_readable_action[1:-1]  # remove parenthesis
            command, param1, param2 = human_readable_action.split(" ")  # split action into relevant parts
            if command == "navigate":
                parsed_plan.append(Navigate(ActionType.NAVIGATE, self.__parse_param(param1), self.__parse_param(param2)))
            elif command == "pickup":
                parsed_plan.append(PickUp(ActionType.PICKUP, self.__parse_param(param1), self.__parse_param(param2)))
            elif command == "dropoff":
                parsed_plan.append(DropOff(ActionType.DROP_OFF, self.__parse_param(param1), self.__parse_param(param2)))

        return parsed_plan

    @staticmethod
    def __parse_param(param: str):
        if param == 'ball_1' or param == 'ball_2' or param == 'ball_3':
            return Item.BALL
        elif param == 'cup_1' or param == 'cup_2' or param == 'cup_3':
            return Item.CUP
        elif param == 'beer_1' or param == 'beer_2' or param == 'beer_3':
            return Item.BEER
        elif param == 'ball_shelf':
            return Location.BALL_SHELF
        elif param == 'cup_shelf':
            return Location.CUP_SHELF
        elif param == 'beer_shelf':
            return Location.BEER_SHELF
        elif param == 'cargo_bay':
            return Location.CARGO_BAY
        elif param == 'starting_location':
            return Location.STARTING_LOCATION




if __name__ == '__main__':
    request = {'cups': 2, 'beers': 2, 'balls': 0}
    planner = Planner('warehouse_domain.pddl', 'problem_template.pddl')
    plan = planner.get_plan(request)
    print(plan)
