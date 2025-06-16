'''
Entry point for the confocal microscope control application.

This script initializes the graphical user interface and processes command-line arguments.
'''
import argparse

from gui import MicroscopeGUI


def parse_args():
    """
    Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments containing the serial port.
    """
    parser = argparse.ArgumentParser(
        description='Confocal microscope control application.'
    )
    parser.add_argument(
        '-p', '--port',
        type=str,
        default='COM4',
        help='Serial port to connect to the Arduino (e.g., COM4 or /dev/ttyUSB0)'
    )
    return parser.parse_args()


def main():
    """
    Main entry point of the application.

    Parses arguments, initializes the GUI, and starts the main event loop.
    """
    args = parse_args()
    # Initialize and run the GUI application
    app = MicroscopeGUI(port=args.port)
    app.root.mainloop()


if __name__ == '__main__':
    main()
