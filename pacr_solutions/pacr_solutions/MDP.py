# MDP.py

####################
# Énumérations alignées avec GetAction.srv
####################
R_OTHER = 0
R_START1 = 1
R_START2 = 2
R_WORKSHOP1 = 3
R_WORKSHOP2 = 4
R_INTERMEDIARY1 = 5
R_INTERMEDIARY2 = 6
R_INTERMEDIARY3 = 7

O_NO_OBJECT = 0
O_START1 = 1
O_START2 = 2
O_CARRIED_UNTRANSFORMED = 3
O_CARRIED_TRANSFORMED = 4

# IMPORTANT : mêmes valeurs que dans .srv
PICK = 0
PLACE = 1
TRANSFORM = 2
WAIT = 3
GOTO_OTHER = 4
GOTO_START1 = 5
GOTO_START2 = 6
GOTO_WORKSHOP1 = 7
GOTO_WORKSHOP2 = 8
GOTO_INTERMEDIARY1 = 9
GOTO_INTERMEDIARY2 = 10
GOTO_INTERMEDIARY3 = 11

class MDP:
    def __init__(self, states, actions, transitions, rewards, gamma=0.9):
        self.states = states
        self.actions = actions
        # transitions[(s, a)] = [(prob, next_state), ...]
        self.transitions = transitions
        # rewards[(s, a, next_state)] = float
        self.rewards = rewards
        self.gamma = gamma

    def R(self, s, a, s_next):
        return self.rewards.get((s, a, s_next), 0.0)

    def T(self, s, a):
        return self.transitions.get((s, a), [])

def value_iteration(mdp: MDP, epsilon=0.01):
    """Algorithme de Value Iteration retournant (policy, V)."""
    V = {s: 0.0 for s in mdp.states}

    while True:
        delta = 0.0
        for s in mdp.states:
            v_old = V[s]
            possible_actions = [a for a in mdp.actions if mdp.T(s, a)]
            if not possible_actions:
                continue

            # On calcule la meilleure espérance de valeur
            V[s] = max(
                sum(
                    p * (mdp.R(s, a, sn) + mdp.gamma * V[sn])
                    for (p, sn) in mdp.T(s, a)
                )
                for a in possible_actions
            )
            delta = max(delta, abs(v_old - V[s]))

        if delta < epsilon:
            break

    # Construire la policy gloutonne
    policy = {}
    for s in mdp.states:
        possible_actions = [a for a in mdp.actions if mdp.T(s, a)]
        if not possible_actions:
            policy[s] = WAIT
            continue
        best_a = max(
            possible_actions,
            key=lambda a: sum(
                p*(mdp.R(s, a, sn) + mdp.gamma * V[sn])
                for (p, sn) in mdp.T(s, a)
            )
        )
        policy[s] = best_a

    return policy, V

def build_mdp():
    """
    Construit un MDP assez complet :
     - Positions (START1, START2, WORKSHOP1, WORKSHOP2, INTERMEDIARY1,2,3)
     - GOTO_* pour relier les lieux "voisins"
     - PICK, PLACE, TRANSFORM, WAIT
     - Pas de "GOTO" sur soi-même dans (R_START1, O_START1) => évite boucle
    """
    # 1) Ensemble des états
    robot_positions = [
        R_OTHER, R_START1, R_START2,
        R_WORKSHOP1, R_WORKSHOP2,
        R_INTERMEDIARY1, R_INTERMEDIARY2, R_INTERMEDIARY3
    ]
    object_states = [
        O_NO_OBJECT, O_START1, O_START2,
        O_CARRIED_UNTRANSFORMED, O_CARRIED_TRANSFORMED
    ]
    states = [(rp, os) for rp in robot_positions for os in object_states]

    # 2) Ensemble des actions
    actions = [
        PICK, PLACE, TRANSFORM, WAIT,
        GOTO_START1, GOTO_START2,
        GOTO_WORKSHOP1, GOTO_WORKSHOP2,
        GOTO_INTERMEDIARY1, GOTO_INTERMEDIARY2, GOTO_INTERMEDIARY3
    ]

    transitions = {}
    rewards = {}

    # Helper : ajout d'une transition GOTO
    def add_goto(from_loc, to_loc, action, cost=-1.0):
        """Ajoute s --action--> s_next pour tout object_state."""
        for o in object_states:
            # Évite la boucle "même endroit" si vous le désirez
            if from_loc == to_loc:
                continue

            s = (from_loc, o)
            s_next = (to_loc, o)
            transitions.setdefault((s, action), []).append((1.0, s_next))
            rewards[(s, action, s_next)] = cost

    # Exemples de connexions GOTO
    add_goto(R_START1, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)
    add_goto(R_INTERMEDIARY2, R_START1, GOTO_START1)

    add_goto(R_START2, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)
    add_goto(R_INTERMEDIARY2, R_START2, GOTO_START2)

    add_goto(R_WORKSHOP1, R_INTERMEDIARY1, GOTO_INTERMEDIARY1)
    add_goto(R_INTERMEDIARY1, R_WORKSHOP1, GOTO_WORKSHOP1)

    add_goto(R_WORKSHOP1, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)
    add_goto(R_INTERMEDIARY2, R_WORKSHOP1, GOTO_WORKSHOP1)

    add_goto(R_WORKSHOP2, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)
    add_goto(R_INTERMEDIARY2, R_WORKSHOP2, GOTO_WORKSHOP2)

    add_goto(R_INTERMEDIARY1, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)
    add_goto(R_INTERMEDIARY2, R_INTERMEDIARY1, GOTO_INTERMEDIARY1)

    add_goto(R_INTERMEDIARY2, R_INTERMEDIARY3, GOTO_INTERMEDIARY3)
    add_goto(R_INTERMEDIARY3, R_INTERMEDIARY2, GOTO_INTERMEDIARY2)

    # 3) PICK
    # (R_START1, O_START1) -> PICK -> (R_START1, O_CARRIED_UNTRANSFORMED)
    # (R_START2, O_START2) -> PICK -> ...
    def add_pick(location, obj):
        s = (location, obj)
        s_next = (location, O_CARRIED_UNTRANSFORMED)
        transitions.setdefault((s, PICK), []).append((1.0, s_next))
        rewards[(s, PICK, s_next)] = -2.0

    add_pick(R_START1, O_START1)
    add_pick(R_START2, O_START2)

    # 4) TRANSFORM : dans un workshop, si on a O_CARRIED_UNTRANSFORMED
    def add_transform(ws):
        s = (ws, O_CARRIED_UNTRANSFORMED)
        s_next = (ws, O_CARRIED_TRANSFORMED)
        transitions.setdefault((s, TRANSFORM), []).append((1.0, s_next))
        rewards[(s, TRANSFORM, s_next)] = -5.0

    add_transform(R_WORKSHOP1)
    add_transform(R_WORKSHOP2)

    # 5) PLACE : si on porte un O_CARRIED_TRANSFORMED, on peut le poser
    def add_place(loc, carried=O_CARRIED_TRANSFORMED):
        s = (loc, carried)
        s_next = (loc, O_NO_OBJECT)
        transitions.setdefault((s, PLACE), []).append((1.0, s_next))
        rewards[(s, PLACE, s_next)] = 0.0

    add_place(R_WORKSHOP1)
    add_place(R_WORKSHOP2)
    add_place(R_START1)
    add_place(R_START2)
    # etc. selon vos besoins

    # 6) WAIT : possible partout (coût -0.5)
    for s in states:
        transitions.setdefault((s, WAIT), []).append((1.0, s))
        rewards[(s, WAIT, s)] = -0.5

    return MDP(states, actions, transitions, rewards, gamma=0.9)
