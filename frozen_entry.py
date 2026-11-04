"""PyInstaller entry point for the ROG Ally DSU server.

Handle multiprocessing worker arguments BEFORE parsing the app's CLI.
This is necessary for the isolated WinRT sensor process in a frozen EXE.
"""
import multiprocessing

if __name__ == '__main__':
    multiprocessing.freeze_support()
    from ally_dsu import main
    main()
