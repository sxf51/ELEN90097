
from automata.fa.dfa import DFA

# Moore FSM
UART_DFA = DFA(
    states={"idle", "start", 
            "bit1_0_even", "bit2_0_even", "bit3_0_even", "bit4_0_even", "bit5_0_even", "bit6_0_even", "bit7_0_even",
            "bit2_1_even", "bit3_1_even", "bit4_1_even", "bit5_1_even", "bit6_1_even", "bit7_1_even", "parity_even", 
            "bit2_0_odd", "bit3_0_odd", "bit4_0_odd", "bit5_0_odd", "bit6_0_odd", "bit7_0_odd", "parity_odd", 
            "bit1_1_odd", "bit2_1_odd", "bit3_1_odd", "bit4_1_odd", "bit5_1_odd", "bit6_1_odd", "bit7_1_odd", 
            "stop"},
    input_symbols={"0", "1"},
    transitions={
        "idle" : {"0" : {"start"}, "1" : {"idle"}},
        "start" : {"0" : {"bit1_0_even"}, "1" : {"bit1_1_odd"}},
        "bit1_0_even" : {"0" : {"bit2_0_even"}, "1" : {"bit2_1_odd"}},
        "bit2_0_even" : {"0" : {"bit3_0_even"}, "1" : {"bit3_1_odd"}},
        "bit3_0_even" : {"0" : {"bit4_0_even"}, "1" : {"bit4_1_odd"}},
        "bit4_0_even" : {"0" : {"bit5_0_even"}, "1" : {"bit5_1_odd"}},
        "bit5_0_even" : {"0" : {"bit6_0_even"}, "1" : {"bit6_1_odd"}},
        "bit6_0_even" : {"0" : {"bit7_0_even"}, "1" : {"bit7_1_odd"}},
        "bit7_0_even" : {"0" : {"parity_even"}, "1" : {"parity_odd"}},
        "bit2_1_even" : {"0" : {"bit3_0_even"}, "1" : {"bit3_1_odd"}},
        "bit3_1_even" : {"0" : {"bit4_0_even"}, "1" : {"bit4_1_odd"}},
        "bit4_1_even" : {"0" : {"bit5_0_even"}, "1" : {"bit5_1_odd"}},
        "bit5_1_even" : {"0" : {"bit6_0_even"}, "1" : {"bit6_1_odd"}},
        "bit6_1_even" : {"0" : {"bit7_0_even"}, "1" : {"bit7_1_odd"}},
        "bit7_1_even" : {"0" : {"parity_even"}, "1" : {"parity_odd"}},
        "parity_even" : {"0" : {"stop"}, "1" : {"idle"}},
        "bit2_0_odd" : {"0" : {"bit3_0_odd"}, "1" : {"bit3_1_even"}},
        "bit3_0_odd" : {"0" : {"bit4_0_odd"}, "1" : {"bit4_1_even"}},
        "bit4_0_odd" : {"0" : {"bit5_0_odd"}, "1" : {"bit5_1_even"}},
        "bit5_0_odd" : {"0" : {"bit6_0_odd"}, "1" : {"bit6_1_even"}},
        "bit6_0_odd" : {"0" : {"bit7_0_odd"}, "1" : {"bit7_1_even"}},
        "bit7_0_odd" : {"0" : {"parity_odd"}, "1" : {"parity_even"}},
        "parity_odd" : {"0" : {"idle"}, "1" : {"stop"}},
        "bit1_1_odd" : {"0" : {"bit2_0_odd"}, "1" : {"bit2_1_even"}},
        "bit2_1_odd" : {"0" : {"bit3_0_odd"}, "1" : {"bit3_1_even"}},
        "bit3_1_odd" : {"0" : {"bit4_0_odd"}, "1" : {"bit4_1_even"}},
        "bit4_1_odd" : {"0" : {"bit5_0_odd"}, "1" : {"bit5_1_even"}},
        "bit5_1_odd" : {"0" : {"bit6_0_odd"}, "1" : {"bit6_1_even"}},
        "bit6_1_odd" : {"0" : {"bit7_0_odd"}, "1" : {"bit7_1_even"}},
        "bit7_1_odd" : {"0" : {"parity_odd"}, "1" : {"parity_even"}},
        "stop" : {"0" : {"start"}, "1" : {"idle"}}
    },
    initial_state="idle",
    final_states={"stop"}
)

UART_DFA.show_diagram()