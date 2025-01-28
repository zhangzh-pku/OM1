import requests
import json

def test_query_endpoint(query_text: str, base_url: str = "http://api.openmind.org"):
    """
    Test the query endpoint with HTTP POST request.
    
    Parameters
    ----------
    query_text : str
        The query to send
    base_url : str
        Base URL of the API
    """
    # Construct the full URL
    url = f"{base_url}/api/core/query"
    
    # Prepare the request payload
    payload = {
        "query": query_text
    }
    
    # Set headers
    headers = {
        "Content-Type": "application/json"
    }
    
    try:
        # Make the POST request
        print(f"Sending request to: {url}")
        print(f"Payload: {payload}")
        
        response = requests.post(url, json=payload, headers=headers)
        
        # Print response status
        print(f"\nStatus Code: {response.status_code}")
        
        # Print response content
        if response.status_code == 200:
            message = response.json()
            documents = message["results"]
            results  = '\n\n'.join([r.get('content', {}).get('text', '') for r in documents if r.get('content', {}).get('text', '')])
            print("\nSuccess! Results:")
            print(json.dumps(results, indent=2))
        else:
            print("\nError Response:")
            print(response.text)
            
    except Exception as e:
        print(f"\nError making request: {e}")

if __name__ == "__main__":
    # Test the endpoint
    test_query = "How does omOS handle actions?"
    test_query_endpoint(test_query)