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

  const setStatus = (message, tone = "info") => {
    if (!statusElement) {
      return;
    }
    statusElement.textContent = message;
    statusElement.classList.remove("text-danger", "text-warning", "text-muted");

    if (tone === "error") {
      statusElement.classList.add("text-danger");
    } else if (tone === "warning") {
      statusElement.classList.add("text-warning");
    } else {
      statusElement.classList.add("text-muted");
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

  const updateStatusTimestamp = (retrievedAt, source = "live") => {
    if (!statusElement) {
      return;
    }

    const timestamp = retrievedAt ? new Date(retrievedAt) : new Date();
    if (Number.isNaN(timestamp.getTime())) {
      const tone = source === "fallback" ? "warning" : "info";
      const message =
        source === "fallback"
          ? "Showing fallback weather conditions."
          : "Latest conditions updated.";
      setStatus(message, tone);
      return;
    }

    const formatter = new Intl.DateTimeFormat(undefined, {
      hour: "numeric",
      minute: "2-digit",
    });

    const formattedTime = formatter.format(timestamp);
    if (source === "fallback") {
      setStatus(
        `Updated ${formattedTime} local time. Showing fallback weather data.`,
        "warning",
      );
    } else {
      setStatus(`Updated ${formattedTime} local time.`, "info");
    }
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
      updateStatusTimestamp(
        payload.retrieved_at,
        payload.source ?? "live",
      );
    } catch (error) {
      console.error("Failed to load weather data", error);
      setStatus("Unable to load latest weather for Macleay Island.", "error");
    } finally {
      window.setTimeout(loadWeather, REFRESH_INTERVAL_MS);
    }
  };

  const startFetching = () => {
    if (!endpoint) {
      setStatus("Weather endpoint is not configured.", "error");
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
