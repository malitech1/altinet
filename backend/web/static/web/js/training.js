const container = document.querySelector("[data-training-endpoint]");

if (container) {
  const endpoint = container.dataset.trainingEndpoint;
  const testingEndpoint = container.dataset.testingEndpoint || "";
  const form = container.querySelector("[data-training-form]");
  const nameInput = form?.querySelector("[data-training-name]");
  const notesInput = form?.querySelector("[data-training-notes]");
  const startButton = form?.querySelector('[data-action="start-camera"]');
  const captureButton = form?.querySelector('[data-action="capture"]');
  const clearButton = form?.querySelector('[data-action="clear"]');
  const testButton = form?.querySelector('[data-action="test"]');
  const trainButton = form?.querySelector('[data-action="train"]');
  const statusElement = form?.querySelector('[data-training-status]');
  const videoElement = form?.querySelector('[data-training-video]');
  const placeholderElement = form?.querySelector('[data-training-placeholder]');
  const capturesContainer = form?.querySelector('[data-training-captures]');
  const emptyMessage = form?.querySelector('[data-training-empty]');
  const captureCanvas = form?.querySelector('[data-training-canvas]');
  const testCanvas = form?.querySelector('[data-test-canvas]');
  const testStatusElement = form?.querySelector('[data-test-status]');
  const testResultContainer = form?.querySelector('[data-test-result]');
  const testResultMessage = form?.querySelector('[data-test-result-message]');

  /** @type {{ dataUrl: string; selected: boolean }} */
  const captures = [];
  let mediaStream = null;
  let testBusy = false;

  const streamHasLiveTracks = (stream) => {
    if (!stream || typeof stream.getTracks !== "function") {
      return false;
    }
    return stream.getTracks().some((track) => track.readyState === "live");
  };

  const monitorStream = (stream, getLatestStream, onEnded) => {
    if (!stream || typeof stream.getTracks !== "function") {
      return;
    }

    stream.getTracks().forEach((track) => {
      track.addEventListener("ended", () => {
        if (getLatestStream() !== stream) {
          return;
        }
        if (streamHasLiveTracks(stream)) {
          return;
        }
        onEnded();
      });
    });
  };

  const getCsrfToken = () => {
    const name = "csrftoken";
    const cookies = document.cookie ? document.cookie.split(";") : [];
    for (const cookie of cookies) {
      const trimmed = cookie.trim();
      if (trimmed.startsWith(`${name}=`)) {
        return decodeURIComponent(trimmed.substring(name.length + 1));
      }
    }
    return null;
  };

  const setStatus = (message, isError = false) => {
    if (!statusElement) {
      return;
    }
    statusElement.textContent = message || "";
    statusElement.classList.toggle("text-danger", Boolean(isError));
    statusElement.classList.toggle("text-muted", Boolean(message) && !isError);
  };

  const setTestStatus = (message, isError = false) => {
    if (!testStatusElement) {
      return;
    }
    testStatusElement.textContent = message || "";
    testStatusElement.classList.toggle("text-danger", Boolean(isError));
    testStatusElement.classList.toggle("text-muted", Boolean(message) && !isError);
  };

  const showTestResult = (message, variant = "info") => {
    if (!testResultContainer || !testResultMessage) {
      return;
    }
    testResultContainer.toggleAttribute("hidden", !message);
    testResultMessage.textContent = message || "";
    testResultMessage.className = `alert alert-${variant} mb-0`;
  };

  const stopCamera = () => {
    if (!mediaStream) {
      return;
    }
    const streamToStop = mediaStream;
    mediaStream = null;
    streamToStop.getTracks().forEach((track) => {
      try {
        track.stop();
      } catch (error) {
        console.warn("Unable to stop media track", error);
      }
    });
    if (videoElement) {
      videoElement.srcObject = null;
      videoElement.setAttribute("hidden", "hidden");
    }
    if (placeholderElement) {
      placeholderElement.removeAttribute("hidden");
    }
    if (captureButton) {
      captureButton.disabled = true;
    }
    updateButtons();
  };

  const updateButtons = () => {
    const hasCaptures = captures.some((capture) => capture.selected);
    if (clearButton) {
      clearButton.disabled = captures.length === 0;
    }
    if (trainButton) {
      const nameFilled = Boolean(nameInput?.value.trim());
      trainButton.disabled = !hasCaptures || !nameFilled;
    }
    if (testButton) {
      const testingAvailable = Boolean(testingEndpoint);
      const cameraSupported = Boolean(navigator.mediaDevices?.getUserMedia);
      testButton.disabled = testBusy || !testingAvailable || !cameraSupported;
    }
  };

  const renderCaptures = () => {
    if (!capturesContainer) {
      return;
    }
    capturesContainer.innerHTML = "";

    if (emptyMessage) {
      emptyMessage.toggleAttribute("hidden", captures.length > 0);
    }

    captures.forEach((capture, index) => {
      const col = document.createElement("div");
      col.className = "col";
      col.innerHTML = `
        <div class="card h-100 shadow-sm ${capture.selected ? "border-primary" : "border-secondary"}">
          <img src="${capture.dataUrl}" class="card-img-top" alt="Captured frame ${index + 1}" />
          <div class="card-body p-2 d-flex flex-column gap-2">
            <div class="form-check">
              <input class="form-check-input" type="checkbox" id="capture-${index}" ${
                capture.selected ? "checked" : ""
              } />
              <label class="form-check-label small" for="capture-${index}">
                Use this photo
              </label>
            </div>
            <button type="button" class="btn btn-outline-danger btn-sm" data-action="remove">
              Remove
            </button>
          </div>
        </div>
      `;

      const checkbox = col.querySelector("input[type='checkbox']");
      const removeButton = col.querySelector("[data-action='remove']");

      if (checkbox) {
        checkbox.addEventListener("change", (event) => {
          capture.selected = event.target.checked;
          renderCaptures();
          updateButtons();
        });
      }

      if (removeButton) {
        removeButton.addEventListener("click", () => {
          captures.splice(index, 1);
          renderCaptures();
          updateButtons();
        });
      }

      capturesContainer.appendChild(col);
    });
  };

  const startCamera = async () => {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      setStatus("Camera access is not supported in this browser.", true);
      return;
    }

    if (mediaStream) {
      if (streamHasLiveTracks(mediaStream)) {
        return;
      }
      stopCamera();
    }

    try {
      mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: "user" },
        audio: false,
      });
      if (videoElement) {
        videoElement.srcObject = mediaStream;
        videoElement.removeAttribute("hidden");
      }
      if (placeholderElement) {
        placeholderElement.setAttribute("hidden", "hidden");
      }
      if (captureButton) {
        captureButton.disabled = false;
      }
      setStatus("Camera ready. Capture a variety of angles.");
      updateButtons();
      monitorStream(mediaStream, () => mediaStream, () => {
        stopCamera();
        setStatus("Camera disconnected. Restart to continue capturing.", true);
      });
    } catch (error) {
      console.error("Unable to start camera", error);
      setStatus("We couldn't access the camera. Check browser permissions and try again.", true);
      updateButtons();
    }
  };

  const captureFrame = () => {
    if (!videoElement || !captureCanvas) {
      return;
    }

    const width = videoElement.videoWidth;
    const height = videoElement.videoHeight;
    if (!width || !height) {
      setStatus("Camera is warming up. Try capturing again in a moment.", true);
      return;
    }

    captureCanvas.width = width;
    captureCanvas.height = height;
    const context = captureCanvas.getContext("2d");
    if (!context) {
      setStatus("Unable to capture image from the camera stream.", true);
      return;
    }

    context.drawImage(videoElement, 0, 0, width, height);

    let dataUrl = "";
    try {
      dataUrl = captureCanvas.toDataURL("image/jpeg", 0.92);
    } catch (error) {
      console.error("Failed to encode capture", error);
      setStatus("We couldn't save that capture. Try again.", true);
      return;
    }

    captures.push({ dataUrl, selected: true });
    renderCaptures();
    updateButtons();
    setStatus("Capture added. Deselect any frames you don't want to use.");
  };

  const clearCaptures = () => {
    captures.splice(0, captures.length);
    renderCaptures();
    updateButtons();
  };

  const submitTraining = async (event) => {
    event.preventDefault();
    if (!form || !trainButton) {
      return;
    }

    const selectedCaptures = captures
      .filter((capture) => capture.selected)
      .map((capture) => capture.dataUrl);

    if (!nameInput || selectedCaptures.length === 0) {
      updateButtons();
      return;
    }

    const payload = {
      full_name: nameInput.value.trim(),
      notes: notesInput?.value.trim() || "",
      images: selectedCaptures,
    };

    if (!payload.full_name) {
      updateButtons();
      return;
    }

    const csrfToken = getCsrfToken();

    trainButton.disabled = true;
    trainButton.classList.add("disabled");
    setStatus("Training in progress… this may take a moment.");

    try {
      const response = await fetch(endpoint, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
          ...(csrfToken ? { "X-CSRFToken": csrfToken } : {}),
        },
        body: JSON.stringify(payload),
      });

      const data = await response.json().catch(() => null);

      if (!response.ok || !data?.success) {
        const errorMessage = data?.error || "Training failed. Please try again.";
        throw new Error(errorMessage);
      }

      setStatus(data.message || "Training complete.");
      form.reset();
      clearCaptures();
      stopCamera();
    } catch (error) {
      console.error("Training request failed", error);
      setStatus(error.message || "Unable to train this profile.", true);
    } finally {
      trainButton.classList.remove("disabled");
      updateButtons();
    }
  };

  const waitForVideoFrame = async () => {
    if (!videoElement) {
      return false;
    }
    if (videoElement.videoWidth && videoElement.videoHeight) {
      return true;
    }

    await new Promise((resolve) => {
      const onLoadedData = () => {
        if (videoElement?.videoWidth && videoElement?.videoHeight) {
          resolve();
        } else {
          resolve();
        }
      };
      videoElement.addEventListener("loadeddata", onLoadedData, { once: true });
    });

    return Boolean(videoElement.videoWidth && videoElement.videoHeight);
  };

  const checkForMatch = async () => {
    if (testBusy) {
      return;
    }
    if (!testingEndpoint) {
      setTestStatus("Testing endpoint unavailable.", true);
      return;
    }
    if (!videoElement || !testCanvas) {
      return;
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      setTestStatus("Camera access is not supported in this browser.", true);
      updateButtons();
      return;
    }

    if (!streamHasLiveTracks(mediaStream)) {
      await startCamera();
    }

    if (!streamHasLiveTracks(mediaStream)) {
      setTestStatus("Camera unavailable. Start the camera and try again.", true);
      return;
    }

    const hasFrame = await waitForVideoFrame();
    if (!hasFrame) {
      setTestStatus("Camera is warming up. Try again shortly.", true);
      return;
    }

    const width = videoElement.videoWidth;
    const height = videoElement.videoHeight;
    const context = testCanvas.getContext("2d");
    if (!context) {
      setTestStatus("Unable to analyse the current frame.", true);
      return;
    }

    testCanvas.width = width;
    testCanvas.height = height;
    context.drawImage(videoElement, 0, 0, width, height);

    let dataUrl = "";
    try {
      dataUrl = testCanvas.toDataURL("image/jpeg", 0.92);
    } catch (error) {
      console.error("Failed to encode frame for testing", error);
      setTestStatus("We couldn't capture that frame. Try again.", true);
      return;
    }

    const csrfToken = getCsrfToken();
    testBusy = true;
    updateButtons();
    setTestStatus("Checking for trained faces…");
    showTestResult("");

    try {
      const response = await fetch(testingEndpoint, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
          ...(csrfToken ? { "X-CSRFToken": csrfToken } : {}),
        },
        body: JSON.stringify({ image: dataUrl }),
      });

      const data = await response.json().catch(() => null);
      if (!response.ok || !data) {
        const errorMessage = data?.error || "Unable to evaluate the capture.";
        throw new Error(errorMessage);
      }

      if (!data.success) {
        throw new Error(data.error || "Face recognition is unavailable right now.");
      }

      const confidence = typeof data.confidence === "number" ? data.confidence : 0;
      if (data.match) {
        const name = data.match.full_name || "a known operator";
        showTestResult(
          `${data.message || "Match found."} Confidence ${confidence.toFixed(2)}.`,
          "success",
        );
        setTestStatus(`Detected ${name} with confidence ${confidence.toFixed(2)}.`);
      } else {
        showTestResult(
          `${data.message || "No trained faces matched."} Confidence ${confidence.toFixed(2)}.`,
          "warning",
        );
        setTestStatus(data.message || "No matches found.");
      }
    } catch (error) {
      console.error("Testing request failed", error);
      showTestResult(error.message || "Unable to test the capture.", "danger");
      setTestStatus(error.message || "Unable to test the capture.", true);
    } finally {
      testBusy = false;
      updateButtons();
    }
  };

  nameInput?.addEventListener("input", updateButtons);
  startButton?.addEventListener("click", startCamera);
  captureButton?.addEventListener("click", captureFrame);
  clearButton?.addEventListener("click", clearCaptures);
  form?.addEventListener("submit", submitTraining);
  testButton?.addEventListener("click", checkForMatch);

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      stopCamera();
    }
  });

  window.addEventListener("beforeunload", () => {
    stopCamera();
  });

  if (!testingEndpoint && testButton) {
    testButton.disabled = true;
    setTestStatus("Testing endpoint unavailable.", true);
  } else if (testingEndpoint && testStatusElement) {
    setTestStatus("Use the test button to scan the live feed for known faces.");
  }

  updateButtons();
}
