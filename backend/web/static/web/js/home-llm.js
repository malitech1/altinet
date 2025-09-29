function getCookie(name) {
  const cookieString = document.cookie;
  if (!cookieString) {
    return null;
  }

  return cookieString
    .split(";")
    .map((cookie) => cookie.trim())
    .filter((cookie) => cookie.startsWith(`${name}=`))
    .map((cookie) => decodeURIComponent(cookie.substring(name.length + 1)))[0] ?? null;
}

function setStatus(statusElement, message, isError = false) {
  if (!statusElement) {
    return;
  }

  statusElement.textContent = message;
  statusElement.classList.toggle("text-danger", isError);
  statusElement.classList.toggle("text-muted", !isError);
}

function initializeLlmForm() {
  const form = document.querySelector("[data-llm-form]");
  if (!form) {
    return;
  }

  const input = form.querySelector("[data-llm-input]");
  const submitButton = form.querySelector("[data-llm-submit]");
  const statusElement = form.querySelector("[data-llm-status]");
  const responseElement = form.parentElement.querySelector("[data-llm-response]");

  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    const prompt = input?.value.trim();
    if (!prompt) {
      setStatus(statusElement, "Enter a prompt to ask the assistant.", true);
      input?.focus();
      return;
    }

    setStatus(statusElement, "Generating response…");
    submitButton.disabled = true;
    submitButton.textContent = "Sending…";

    try {
      const response = await fetch("/api/llm/prompt/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-CSRFToken": getCookie("csrftoken") ?? "",
        },
        body: JSON.stringify({ prompt }),
      });

      if (!response.ok) {
        throw new Error(`Assistant returned ${response.status}`);
      }

      const data = await response.json();
      const assistantResponse = data.response?.trim();
      if (assistantResponse) {
        responseElement.textContent = assistantResponse;
      } else {
        responseElement.textContent = "The assistant did not return any content.";
      }
      setStatus(statusElement, "Response ready.");
    } catch (error) {
      console.error("Failed to fetch assistant response", error);
      setStatus(statusElement, "We couldn't reach the assistant. Please try again.", true);
    } finally {
      submitButton.disabled = false;
      submitButton.textContent = "Send to assistant";
    }
  });
}

document.addEventListener("DOMContentLoaded", initializeLlmForm);
