"""Startup script for the Altinet local node GUI."""

import os
import subprocess


def _configure_network() -> None:
    """Configure networking for the local node.

    The node first connects to the home Wi-Fi network and then creates a
    separate access point for Altinet devices. Internet connectivity is
    blocked on the Altinet network to keep it isolated from the home network.
    Network credentials and interface names are provided via environment
    variables:

    ``ALTINET_HOME_WIFI_SSID`` and ``ALTINET_HOME_WIFI_PASSWORD`` define the
    home network to connect to. ``ALTINET_AP_SSID`` and
    ``ALTINET_AP_PASSWORD`` set the SSID and passphrase for the Altinet access
    point. ``ALTINET_HOME_WIFI_IFACE`` and ``ALTINET_AP_IFACE`` optionally
    specify the wireless interfaces to use (default ``wlan0`` and ``wlan1``
    respectively).

    This function relies on the presence of ``nmcli`` for network management
    and ``iptables`` for blocking forwarding between interfaces.
    """

    home_ssid = os.getenv("ALTINET_HOME_WIFI_SSID")
    home_password = os.getenv("ALTINET_HOME_WIFI_PASSWORD")
    ap_ssid = os.getenv("ALTINET_AP_SSID", "Altinet")
    ap_password = os.getenv("ALTINET_AP_PASSWORD", "altinetpass")
    home_iface = os.getenv("ALTINET_HOME_WIFI_IFACE", "wlan0")
    ap_iface = os.getenv("ALTINET_AP_IFACE", "wlan1")

    if not home_ssid or not home_password:
        raise RuntimeError(
            "Home WiFi credentials must be provided via environment variables"
        )

    # Connect to the home Wi-Fi network
    subprocess.run(
        [
            "nmcli",
            "dev",
            "wifi",
            "connect",
            home_ssid,
            "password",
            home_password,
            "ifname",
            home_iface,
        ],
        check=True,
    )

    # Create an access point for Altinet devices
    subprocess.run(
        [
            "nmcli",
            "dev",
            "wifi",
            "hotspot",
            "ifname",
            ap_iface,
            "ssid",
            ap_ssid,
            "password",
            ap_password,
        ],
        check=True,
    )

    # Prevent forwarding between the Altinet AP and the home network
    subprocess.run(
        ["iptables", "-I", "FORWARD", "-i", ap_iface, "-j", "DROP"],
        check=True,
    )


def start_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    """Start the Django development server for the local node GUI.

    Parameters
    ----------
    host:
        Hostname to bind the server to.
    port:
        Port number for the server.
    """
    try:
        from django.core.management import call_command
        import django
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("Django must be installed to run the local node GUI") from exc

    _configure_network()

    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "altinet.localnode.settings")
    django.setup()
    # Ensure the database and tables exist before starting the server
    call_command("migrate", interactive=False)
    call_command("runserver", f"{host}:{port}")
