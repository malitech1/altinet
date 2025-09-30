const container = document.querySelector("[data-weather-endpoint]");

if (container) {
  const endpoint = container.dataset.weatherEndpoint;
  const statusElement = container.querySelector("[data-weather-status]");
  const REFRESH_INTERVAL_MS = 10 * 60 * 1000;

  const fieldElements = Array.from(
    container.querySelectorAll("[data-weather-field]")
  );

  const asNumber = (value) => {
    if (typeof value === "number") {
      return value;
    }
    if (typeof value === "string" && value.trim() !== "") {
      const parsed = Number.parseFloat(value);
      if (Number.isFinite(parsed)) {
        return parsed;
      }
    }
    return null;
  };

  const formatters = {
    temperature(value) {
      const numeric = asNumber(value);
      if (numeric === null) {
        return "—";
      }
      return `${numeric.toFixed(1)}°C`;
    },
    percentage(value) {
      const numeric = asNumber(value);
      if (numeric === null) {
        return "—";
      }
      return `${Math.round(numeric)}%`;
    },
    "wind-speed"(value) {
      const numeric = asNumber(value);
      if (numeric === null) {
        return "—";
      }
      return `${numeric.toFixed(1)} km/h`;
    },
    "wind-direction"(value) {
      if (!value) {
        return "";
      }
      return `(${value})`;
    },
    degrees(value) {
      const numeric = asNumber(value);
      if (numeric === null) {
        return "—";
      }
      return `${Math.round(numeric)}°`;
    },
    aqi(value) {
      const numeric = asNumber(value);
      if (numeric === null) {
        return "—";
      }
      return `AQI ${Math.round(numeric)}`;
    },
    text(value) {
      if (value === null || value === undefined || value === "") {
        return "—";
      }
      return String(value);
    },
  };

  const setStatus = (message, isError = false) => {
    if (!statusElement) {
      return;
    }
    statusElement.textContent = message;
    if (isError) {
      statusElement.classList.add("text-danger");
      statusElement.classList.remove("text-muted");
    } else {
      statusElement.classList.remove("text-danger");
      if (!statusElement.classList.contains("text-muted")) {
        statusElement.classList.add("text-muted");
      }
    }
  };

  const applyWeatherData = (data) => {
    fieldElements.forEach((element) => {
      const field = element.dataset.weatherField;
      const format = element.dataset.weatherFormat || "text";
      const rawValue = data[field];
      const formatter = formatters[format] || formatters.text;

      if (format === "wind-direction") {
        if (!rawValue) {
          element.textContent = "";
          element.setAttribute("hidden", "hidden");
          return;
        }
        element.removeAttribute("hidden");
        element.textContent = formatter(rawValue);
        return;
      }

      if (rawValue === null || rawValue === undefined || rawValue === "") {
        element.textContent = "—";
        return;
      }

      try {
        element.textContent = formatter(rawValue);
      } catch (error) {
        console.error("Unable to format weather field", field, error);
        element.textContent = String(rawValue);
      }
    });
  };

  const updateStatusTimestamp = (retrievedAt) => {
    if (!statusElement) {
      return;
    }

    const timestamp = retrievedAt ? new Date(retrievedAt) : new Date();
    if (Number.isNaN(timestamp.getTime())) {
      setStatus("Latest conditions updated.");
      return;
    }

    const formatter = new Intl.DateTimeFormat(undefined, {
      hour: "numeric",
      minute: "2-digit",
    });

    setStatus(`Updated ${formatter.format(timestamp)} local time.`);
  };

  const loadWeather = async () => {
    if (!endpoint) {
      return;
    }

    try {
      const response = await fetch(endpoint, {
        headers: { Accept: "application/json" },
      });

      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }

      const payload = await response.json();
      if (!payload.success || !payload.data) {
        throw new Error("Response did not include weather data");
      }

      applyWeatherData(payload.data);
      updateStatusTimestamp(payload.retrieved_at);
    } catch (error) {
      console.error("Failed to load weather data", error);
      setStatus("Unable to load latest weather for Macleay Island.", true);
    } finally {
      window.setTimeout(loadWeather, REFRESH_INTERVAL_MS);
    }
  };

  const startFetching = () => {
    if (!endpoint) {
      setStatus("Weather endpoint is not configured.", true);
      return;
    }
    loadWeather();
  };

  if ("requestIdleCallback" in window) {
    window.requestIdleCallback(startFetching);
  } else {
    window.setTimeout(startFetching, 150);
  }
}
