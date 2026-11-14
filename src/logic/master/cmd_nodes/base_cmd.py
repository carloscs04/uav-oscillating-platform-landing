# File: src/logic/master/cmd_nodes/base_cmd.py

import inspect
from typing import Any, Callable, Dict, List, Set, Tuple
from src.logic.master.master_state import MasterState
from src.logic.master.command_guard import CommandGuard


class BaseNodeCLI:
    """Base CLI router that owns all command words, aliases, and direct Manager method calls for a node."""

    RESERVED_GLOBAL_WORDS: Set[str] = {"start", "stop", "status", "help", "methods", "is_healthy"}

    def __init__(self, node_name: str, manager: Any, state: MasterState, guard: CommandGuard):
        self.node_name = node_name.lower()
        self.manager = manager
        self.state = state
        self.guard = guard

        # Maps command_word -> (handler_fn, description, allow_standalone_in_cmd)
        self._commands: Dict[str, Tuple[Callable[[List[str]], str], str, bool]] = {}

        # Built-in per-node commands
        self.register_cmd("help", self.cmd_help, "Lists available commands for this node", global_alias=False)
        self.register_cmd("status", self.cmd_status, "Displays node status", global_alias=False)
        self.register_cmd("methods", self.cmd_list_methods, "Lists all internal Manager methods", global_alias=False)

        # Register node-specific commands defined in subclass
        self._register_commands()

    def _register_commands(self):
        """Override in subclasses to register all node-specific command words and aliases."""
        pass

    def register_cmd(
        self,
        word: str,
        handler: Callable[[List[str]], str],
        description: str = "",
        aliases: List[str] = None,
        global_alias: bool = True
    ):
        """Registers a primary command word and optional aliases for this node."""
        all_words = [word.lower()] + [a.lower() for a in (aliases or [])]
        for w in all_words:
            can_be_global = global_alias and (w not in self.RESERVED_GLOBAL_WORDS)
            self._commands[w] = (handler, description, can_be_global)

    def owns_command(self, word: str) -> bool:
        """Returns True if MasterNode should route a standalone command word directly to this node."""
        w = word.lower()
        if w in self._commands:
            return self._commands[w][2]

        if w not in self.RESERVED_GLOBAL_WORDS and not w.startswith("_"):
            fn = getattr(self.manager, w, None)
            if callable(fn):
                return True
        return False

    def log(self, msg: str, tag: str = "INFO"):
        self.state.append_cli_log(f"[{self.node_name.upper()}] {msg}", tag)

    def execute_cli_command(self, args: List[str]) -> str:
        """Executes a registered node word or falls back to calling the exact method on self.manager."""
        if not args:
            return self.cmd_status([])

        sub_verb = args[0].lower()
        sub_args = args[1:]

        # 1. Explicit registered word / alias in this cmd_node file
        if sub_verb in self._commands:
            handler, _desc, _global = self._commands[sub_verb]
            return handler(sub_args)

        # 2. Direct fallback to ANY public method inside self.manager
        mgr_fn = getattr(self.manager, sub_verb, None)
        if callable(mgr_fn) and not sub_verb.startswith("_"):
            return self._invoke_manager_method(sub_verb, mgr_fn, sub_args)

        self.log(f"Unknown command '{sub_verb}'. Type '{self.node_name} help' or '{self.node_name} methods'.", "ERROR")
        return "UNKNOWN_SUBCMD"

    def cmd_help(self, _args: List[str] = None) -> str:
        lines = []
        seen_handlers = set()
        for word, (handler, desc, is_global) in self._commands.items():
            if handler in seen_handlers:
                continue
            seen_handlers.add(handler)
            aliases = [w for w, (h, _, _) in self._commands.items() if h == handler and w != word]
            alias_txt = f" (aliases: {', '.join(aliases)})" if aliases else ""
            scope_txt = " [direct]" if is_global else ""
            lines.append(f"  - {word}{alias_txt}{scope_txt}: {desc}")

        self.log("Registered commands:\n" + "\n".join(lines), "INFO")
        return "HELP_OK"

    def cmd_status(self, _args: List[str]) -> str:
        running = getattr(self.manager, "running", False)
        connected = getattr(self.manager, "connected", False)
        self.log(f"running={running} | connected={connected}", "STATUS")
        return "STATUS_OK"

    def cmd_list_methods(self, _args: List[str] = None) -> str:
        """Inspects all callable public methods available in the internal Manager."""
        entries = []
        for attr_name in dir(self.manager):
            if attr_name.startswith("_"):
                continue
            attr = getattr(self.manager, attr_name, None)
            if callable(attr):
                try:
                    sig = str(inspect.signature(attr))
                except Exception:
                    sig = "()"
                entries.append(f"{attr_name}{sig}")

        self.log(f"Methods in {type(self.manager).__name__}: {', '.join(entries)}", "INFO")
        return "METHODS_OK"

    def _invoke_manager_method(self, name: str, fn: Any, raw_args: List[str]) -> str:
        parsed_args = [self._parse_arg(a) for a in raw_args]
        try:
            result = fn(*parsed_args)
            self.log(f"Executed {name}({', '.join(map(str, parsed_args))}) -> {result}", "OK")
            return str(result)
        except TypeError as e:
            self.log(f"Argument mismatch in '{name}': {e}", "ERROR")
            return "ARG_ERROR"
        except Exception as e:
            self.log(f"Execution error in '{name}': {e}", "ERROR")
            return "EXEC_ERROR"

    @staticmethod
    def _parse_arg(token: str) -> Any:
        low = token.lower()
        if low == "true":
            return True
        if low == "false":
            return False
        try:
            if "." in token:
                return float(token)
            return int(token)
        except ValueError:
            return token