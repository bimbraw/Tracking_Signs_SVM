from enum import Enum

# state.py

class State(object):
    """
    Define the state and other utility functions

    """

    def __init__(self):
        print('Initializing robot state in:  ', str(self))

    def on_event(self, event):
        """
        Handles events for the state

        :param self:
        :param event:
        :return:
        """
        pass

    def __repr__(self):
        """
        Used __str__ to describe state
        :param self:
        :return:
        """
        return self.__str__()

    def __str__(self):
        """
        Returns name of string
        :param self:
        :return:
        """

        return self.__class__.__name__


class Events(Enum):
    system_activated = 'system_activated'
    sign_found = 'sign_found'
    at_sign = 'at_sign'
    sign_classified = 'sign_classified'
    action_completed = 'action_completed'
    goal_reached = 'goal_reached'
    none = 'none'


class IdleState(State):
    """

   Initial State with velocity zero waiting for any data to be returned

    """

    def on_event(self, event):
        if event == Events.system_activated:
            return ExploreState()

        return self


class ExploreState(State):
    """

    State that searched for a sign to chase, or follows a random walk to move closer to goal

    """

    def on_event(self, event):
        if event == Events.sign_found:
            return ChaseState()

        return self


class ChaseState(State):
    """

    State for chasing down the sign

    """

    def on_event(self, event):
        if event == Events.at_sign:
            return ClassifyState()

        return self


class ClassifyState(State):
    """

    State for classifying image and completing action

    """

    def on_event(self, event):
        if event == Events.action_completed:
            return ExploreState()
        elif event == Events.goal_reached:
            return GoalState()

        return self


class GoalState(State):
    """

    State to stop when goal is reached

    """


class States(Enum):
    Idle = 0
    Explore = 1
    Chase = 2
    Classify = 3
    Move = 4
    Goal = 5


