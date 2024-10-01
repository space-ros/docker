import json
from openai import OpenAI
import subprocess
import os
import sys
from datetime import datetime

def load_config(file_path):
    try:
        with open(file_path, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: {file_path} not found.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error: {file_path} is not a valid JSON file.")
        sys.exit(1)

def load_prompt(file_path):
    try:
        with open(file_path, "r") as f:
            return f.read()
    except FileNotFoundError:
        print(f"Error: {file_path} not found.")
        sys.exit(1)

def get_chat_response(client, prompt, model):
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": prompt},
            ]
        )
        return response.choices[0].message.content.strip()
    except Exception as e:
        print(f"Error getting response from GPT: {e}")
        sys.exit(1)

def remove_backticks(code_str):
    if code_str.startswith("```python"):
        code_str = code_str[len("```python"):].strip()
    if code_str.endswith("```"):
        code_str = code_str[:-len("```")].strip()
    return code_str

def generate_python_script(res):
    python_code = remove_backticks(res)
    current_time = datetime.now().strftime("%Y%m%d_%H%M")
    filename = f"gen_script_{current_time}.py"
    with open(filename, "w") as f:
        f.write(python_code)
    return filename

def run_python_script(filename):
    result = subprocess.run(["python3", filename], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error executing script: {result.stderr}")
    else:
        print(f"Script output: {result.stdout}")

def main():
    config = load_config('config.json')
    client = OpenAI(api_key=config['openai_api_key'])
    
    pre_prompt = load_prompt("prompt.txt")
    print("Pre-prompt loaded successfully")

    print("Please input command like 'Go forward 10 seconds' and push enter-key")
    user_input = input("")
    prompt = pre_prompt + user_input

    res = get_chat_response(client, prompt, config['openai_model'])
    filename = generate_python_script(res)
    run_python_script(filename)

if __name__ == "__main__":
    main()

