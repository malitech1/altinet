const timeElement = document.getElementById("local-time-display");
const dateElement = document.getElementById("local-date-display");

if (timeElement && dateElement) {
  const updateLocalTime = () => {
    const now = new Date();

    const timeFormatter = new Intl.DateTimeFormat(undefined, {
      hour: "numeric",
      minute: "2-digit",
    });

    const dateFormatter = new Intl.DateTimeFormat(undefined, {
      weekday: "long",
      month: "long",
      day: "numeric",
    });

    timeElement.textContent = timeFormatter.format(now);
    dateElement.textContent = dateFormatter.format(now);
  };

  updateLocalTime();
  window.setInterval(updateLocalTime, 30_000);
}
